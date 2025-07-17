use crate::utils;
use rapier::geometry::SolverFlags;
use rapier::pipeline::{ContactModificationContext, PairFilterContext, PhysicsHooks};
use rapier2d::prelude::Point;
use wasm_bindgen::prelude::*;

pub struct RawPhysicsHooks {
    pub this: js_sys::Object,
    pub filter_contact_pair: js_sys::Function,
    pub filter_intersection_pair: js_sys::Function,
    pub modify_solver_contacts: js_sys::Function,
}

// HACK: the RawPhysicsHooks is no longer Send+Sync because the JS objects are
//       no longer Send+Sync since https://github.com/rustwasm/wasm-bindgen/pull/955
//       As far as this is confined to the bindings this should be fine since we
//       never use threading in wasm.
unsafe impl Send for RawPhysicsHooks {}
unsafe impl Sync for RawPhysicsHooks {}

#[wasm_bindgen]
extern "C" {
    // Use `js_namespace` here to bind `console.log(..)` instead of just
    // `log(..)`
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

fn combine_f32_to_f64(a: f32, b: f32) -> f64 {
    // 将 f32 转换为 u32
    let a_bits = a.to_bits() as u64;
    let b_bits = b.to_bits() as u64;

    // 将两个 u32 合并为一个 u64
    let combined_bits = (a_bits << 32) | b_bits;

    // 将 u64 转换为 f64
    f64::from_bits(combined_bits)
}

impl PhysicsHooks for RawPhysicsHooks {
    fn filter_contact_pair(&self, ctxt: &PairFilterContext) -> Option<SolverFlags> {
        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        let result = self
            .filter_contact_pair
            .bind2(
                &self.this,
                &JsValue::from(utils::flat_handle(ctxt.collider1.0)),
                &JsValue::from(utils::flat_handle(ctxt.collider2.0)),
            )
            .call2(&self.this, &rb1, &rb2)
            .ok()?;
        let flags = result.as_f64()?;
        // TODO: not sure exactly why we have to do `flags as u32` instead
        //       of `flags.to_bits() as u32`.
        SolverFlags::from_bits(flags as u32)
    }

    fn filter_intersection_pair(&self, ctxt: &PairFilterContext) -> bool {
        let rb1 = ctxt
            .rigid_body1
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);
        let rb2 = ctxt
            .rigid_body2
            .map(|rb| JsValue::from(utils::flat_handle(rb.0)))
            .unwrap_or(JsValue::NULL);

        self.filter_intersection_pair
            .bind2(
                &self.this,
                &JsValue::from(utils::flat_handle(ctxt.collider1.0)),
                &JsValue::from(utils::flat_handle(ctxt.collider2.0)),
            )
            .call2(&self.this, &rb1, &rb2)
            .ok()
            .and_then(|res| res.as_bool())
            .unwrap_or(false)
    }

    #[cfg(feature = "dim2")]
    fn modify_solver_contacts(&self, ctxt: &mut ContactModificationContext) {
        if self.modify_solver_contacts.is_null() {
            return;
        }

        let nx = &ctxt.normal.x;
        let ny = &ctxt.normal.y;
        let n = &combine_f32_to_f64(*nx, *ny);

        let default_point = Point::new(0.0, 0.0);
        let point1 = ctxt
            .solver_contacts
            .get(0)
            .map_or(default_point, |contact| contact.point);
        let p1x = &point1.x;
        let p1y = &point1.y;
        let p1 = &combine_f32_to_f64(*p1x, *p1y);

        let point2 = ctxt
            .solver_contacts
            .get(1)
            .map_or(default_point, |contact| contact.point);
        let p2x = &point2.x;
        let p2y = &point2.y;
        let p2 = &combine_f32_to_f64(*p2x, *p2y);

        let ret = self
            .modify_solver_contacts
            .bind2(
                &self.this,
                &JsValue::from(utils::flat_handle(ctxt.collider1.0)),
                &JsValue::from(utils::flat_handle(ctxt.collider2.0)),
            )
            .call3(
                &self.this,
                &JsValue::from_f64(*n),
                &JsValue::from(*p1),
                &JsValue::from(*p2),
            )
            .ok()
            .and_then(|res| res.as_bool())
            .unwrap_or(true);

        if !ret {
            ctxt.solver_contacts.clear();
        }
    }
}

/* NOTE: the following is an attempt to make contact modification work.
 *
#[wasm_bindgen]
#[derive(Copy, Clone, Debug)]
pub struct RawContactManifold(*const ContactManifold);
pub struct RawSolverContact(*const SolverContact);

#[wasm_bindgen]
pub struct RawContactModificationContext {
    pub collider1: u32,
    pub collider2: u32,
    pub rigid_body1: Option<u32>,
    pub rigid_body2: Option<u32>,
    pub manifold: *const ContactManifold,
    pub solver_contacts: *mut Vec<SolverContact>,
    normal: *mut Vector<Real>,
    user_data: *mut u32,
}

#[wasm_bindgen]
impl RawContactModificationContext {
    pub fn collider1(&self) -> u32 {
        self.collider1
    }

    pub fn collider2(&self) -> u32 {
        self.collider2
    }

    #[wasm_bindgen(getter)]
    pub fn normal(&self) -> RawVector {
        unsafe { RawVector(*self.normal) }
    }

    #[wasm_bindgen(setter)]
    pub fn set_normal(&mut self, normal: RawVector) {
        unsafe {
            *self.normal = normal.0;
        }
    }

    #[wasm_bindgen(getter)]
    pub fn user_data(&self) -> u32 {
        unsafe { *self.user_data }
    }

    #[wasm_bindgen(setter)]
    pub fn set_user_data(&mut self, user_data: u32) {
        unsafe {
            *self.user_data = user_data;
        }
    }

    pub fn num_solver_contacts(&self) -> usize {
        unsafe { (*self.solver_contacts).len() }
    }

    pub fn clear_solver_contacts(&mut self) {
        unsafe { (*self.solver_contacts).clear() }
    }

    pub fn remove_solver_contact(&mut self, i: usize) {
        unsafe {
            if i < self.num_solver_contacts() {
                (*self.solver_contacts).swap_remove(i);
            }
        }
    }

    pub fn solver_contact_point(&self, i: usize) -> Option<RawVector> {
        unsafe {
            (*self.solver_contacts)
                .get(i)
                .map(|c| c.point.coords.into())
        }
    }

    pub fn set_solver_contact_point(&mut self, i: usize, pt: &RawVector) {
        unsafe {
            if let Some(c) = (*self.solver_contacts).get_mut(i) {
                c.point = pt.0.into()
            }
        }
    }

    pub fn solver_contact_dist(&self, i: usize) -> Real {
        unsafe {
            (*self.solver_contacts)
                .get(i)
                .map(|c| c.dist)
                .unwrap_or(0.0)
        }
    }

    pub fn set_solver_contact_dist(&mut self, i: usize, dist: Real) {
        unsafe {
            if let Some(c) = (*self.solver_contacts).get_mut(i) {
                c.dist = dist
            }
        }
    }

    pub fn solver_contact_friction(&self, i: usize) -> Real {
        unsafe { (*self.solver_contacts)[i].friction }
    }

    pub fn set_solver_contact_friction(&mut self, i: usize, friction: Real) {
        unsafe {
            if let Some(c) = (*self.solver_contacts).get_mut(i) {
                c.friction = friction
            }
        }
    }

    pub fn solver_contact_restitution(&self, i: usize) -> Real {
        unsafe { (*self.solver_contacts)[i].restitution }
    }

    pub fn set_solver_contact_restitution(&mut self, i: usize, restitution: Real) {
        unsafe {
            if let Some(c) = (*self.solver_contacts).get_mut(i) {
                c.restitution = restitution
            }
        }
    }

    pub fn solver_contact_tangent_velocity(&self, i: usize) -> RawVector {
        unsafe { (*self.solver_contacts)[i].tangent_velocity.into() }
    }

    pub fn set_solver_contact_tangent_velocity(&mut self, i: usize, vel: &RawVector) {
        unsafe {
            if let Some(c) = (*self.solver_contacts).get_mut(i) {
                c.tangent_velocity = vel.0.into()
            }
        }
    }
}
*/
