use liealg::{Group, SE3};
use nalgebra::{Matrix3, Matrix6};

pub(super) fn to_local_spatial_inertial(
    inertia_frame: &SE3<f64>,
    inertia: &Matrix3<f64>,
    mass: f64,
) -> Matrix6<f64> {
    let mut i_b = Matrix6::from_diagonal_element(mass);
    i_b.fixed_view_mut::<3, 3>(0, 0).copy_from(inertia);
    let b_t_a = inertia_frame.inv();
    let adj_b_t_a = Matrix6::from_column_slice(b_t_a.adjoint().as_slice());
    adj_b_t_a.transpose() * i_b * adj_b_t_a
}

#[cfg(test)]
mod tests {
    use liealg::{SE3, SO3};

    use crate::spatial_inertial::to_local_spatial_inertial;

    #[test]
    fn to_local_spatial_inertial_test() {
        use nalgebra::Matrix3;

        let inertia_frame = SE3::new(&SO3::identity(), [1., 0., 0.]);
        let inertia = Matrix3::from_diagonal_element(4.);
        let mass = 5.0;
        let spatial_inertial = to_local_spatial_inertial(&inertia_frame, &inertia, mass);
        println!("spatial_inertial: {:.3}", spatial_inertial);
    }
}
