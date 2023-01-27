//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBEXPLICITENGINE_H
#define A5_RBEXPLICITENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with explicit integration scheme.
 */
class RBExplicitEngine : public RBEngine {
public:
    RBExplicitEngine() : RBEngine() {}

    ~RBExplicitEngine() override = default;

    void step(double dt) override {
        // update external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing velocity.
            //
            // Hint:
            // - complete the function,
            // Quaternion updateRotationGivenAngularVelocity(const Quaternion &q, const V3D &angularVelocity, double dt)
            // in src/libs/sim/include/sim/RBEngine.h and use it for updating orientation of rigidbody.
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame.

            double mass = rb->rbProps.mass;
            Matrix3x3 rotmat = rb->state.orientation.normalized().toRotationMatrix();
            Matrix3x3 MOI_local = rb->rbProps.MOI_local;
            Matrix3x3 MOI_world = rotmat * MOI_local * rotmat.transpose();

            V3D v_i = rb->state.velocity;
            V3D w_i = rb->state.angularVelocity;


            rb->state.velocity += dt*f/mass;
            rb->state.angularVelocity += dt*MOI_world.inverse()*(tau - rb->state.angularVelocity.cross(V3D(MOI_world * rb->state.angularVelocity)));

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing pose.
            rb->state.pos = rb->state.pos + v_i * dt;  // TODO: change this!
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, w_i, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBEXPLICITENGINE_H
