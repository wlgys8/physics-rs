type Isometry3 = nalgebra::Isometry3<f32>;
type Point3 = nalgebra::Point3<f32>;

pub struct SphereCollider {
    pub radius: f32,
}

pub enum Collider {
    Sphere(SphereCollider),
}

impl From<SphereCollider> for Collider {
    #[inline]
    fn from(sphere: SphereCollider) -> Self {
        Self::Sphere(sphere)
    }
}

pub struct TransformedCollider {
    pub collider: Collider,
    pub transform: Isometry3,
}

pub trait ComputeCollisionWithPoint {
    fn compute_collision_with_point(
        &self,
        collider_transform: Isometry3,
        point: Point3,
    ) -> Option<Point3>;
}

impl ComputeCollisionWithPoint for SphereCollider {
    fn compute_collision_with_point(
        &self,
        collider_transform: Isometry3,
        point: Point3,
    ) -> Option<Point3> {
        let center: Point3 = collider_transform.translation.vector.into();
        let dir = point - center;
        let distance = dir.magnitude();
        if distance >= self.radius {
            None
        } else {
            Some(center + dir / distance * self.radius)
        }
    }
}

impl TransformedCollider {
    #[inline]
    pub fn compute_collision_with_point(&self, point: Point3) -> Option<Point3> {
        match &self.collider {
            Collider::Sphere(sphere) => sphere.compute_collision_with_point(self.transform, point),
        }
    }
}
