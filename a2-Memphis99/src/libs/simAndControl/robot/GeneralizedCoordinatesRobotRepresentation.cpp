#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <utils/utils.h>

namespace crl {

GeneralizedCoordinatesRobotRepresentation::
    GeneralizedCoordinatesRobotRepresentation(Robot *a) {
    robot = a;
    resize(q, getDOFCount());
    syncGeneralizedCoordinatesWithRobotState();
}

//returns the total number of degrees of freedom in the reduced representation
int GeneralizedCoordinatesRobotRepresentation::getDOFCount() const {
    //we will have 3 for root position, 3 for root orientation, and then 1 for each joint,
    //assuming they're all hinge joints, which is what this data structure is designed for
    return 6 + (int)robot->jointList.size();
}

//returns the index of the generalized coordinate corresponding to this joint
int GeneralizedCoordinatesRobotRepresentation::getQIdxForJoint(
    RBJoint *joint) const {
    //the root of the robot has a nullptr joint, and as such this should correspond to the first qIdx of the root DOFs
    if (joint == NULL) return 5;
    return 6 + joint->jIndex;
}

//returns the index of the generalized coordinate corresponding to the index of this joint
int GeneralizedCoordinatesRobotRepresentation::getQIdxForJointIdx(
    int jIdx) const {
    return 6 + jIdx;
}

//returns a pointer to the joint corresponding to this generalized coordinate index.
//If the index corresponds to a root DOF, this method will return NULL.
RBJoint *GeneralizedCoordinatesRobotRepresentation::getJointForQIdx(
    int qIdx) const {
    if (qIdx < 6) return NULL;
    return robot->jointList[qIdx - 6];
}

/**
* In the tree-like hierarchy of joints/dofs, this method returns the parent index of the
* dof corresponding to qIdx
*/
int GeneralizedCoordinatesRobotRepresentation::getParentQIdxOf(int qIdx) {
    if (qIdx < 6) return qIdx - 1;
    return getQIdxForJoint(getJointForQIdx(qIdx)->parent->pJoint);
}

// updates q and qDot given current state of robot
void GeneralizedCoordinatesRobotRepresentation::
    syncGeneralizedCoordinatesWithRobotState() {
    // write out the position of the root...
    RobotState state(robot);
    P3D pos = state.getPosition();
    Quaternion orientation = state.getOrientation();

    q[0] = pos.x;
    q[1] = pos.y;
    q[2] = pos.z;

    // Root
    computeEulerAnglesFromQuaternion(orientation, getQAxis(5), getQAxis(4),
                                     getQAxis(3), q[5], q[4], q[3]);

    // Now go through each joint, and decompose it as appropriate...
    for (uint i = 0; i < robot->jointList.size(); i++) {
        int qIdx = getQIdxForJointIdx(i);
        // Only 1-dof hinge joints
        computeRotationAngleFromQuaternion(state.getJointRelativeOrientation(i),
                                           getQAxis(qIdx), q[qIdx]);
    }
}

// returns the axis corresponding to the indexed generalized coordinate,
// expressed in local coordinates
V3D GeneralizedCoordinatesRobotRepresentation::getQAxis(int qIndex) const {
    if (qIndex >= 0 || qIndex < 6) {
        // the first three are the translational dofs of the body
        if (qIndex == 0) return V3D(1, 0, 0);
        if (qIndex == 1) return V3D(0, 1, 0);
        if (qIndex == 2) return V3D(0, 0, 1);
        if (qIndex == 3) return RBGlobals::worldUp;  // y - yaw
        if (qIndex == 4)
            return RBGlobals::worldUp.cross(robot->forward);  // x - pitch
        if (qIndex == 5) return robot->forward;               // z - roll
    }

    return getJointForQIdx(qIndex)->rotationAxis;
}

void GeneralizedCoordinatesRobotRepresentation::
    syncRobotStateWithGeneralizedCoordinates() {
    RobotState rs(robot);
    getReducedRobotState(rs);
    robot->setState(&rs);
}

// given the current state of the generalized representation, output the reduced
// state of the robot
void GeneralizedCoordinatesRobotRepresentation::getReducedRobotState(
    RobotState &state) {
    // set the position, velocity, rotation and angular velocity for the root
    state.setPosition(P3D(0, 0, 0) + getQAxis(0) * q[0] + getQAxis(1) * q[1] +
                      getQAxis(2) * q[2]);
    state.setOrientation(getOrientationFor(robot->root));

    for (uint i = 0; i < robot->jointList.size(); i++) {
        int qIdx = getQIdxForJointIdx(i);
        Quaternion jointOrientation =
            getRotationQuaternion(q[qIdx], getQAxis(qIdx));

        state.setJointRelativeOrientation(jointOrientation, i);
    }
    // and done...
}

// sets the current q values
void GeneralizedCoordinatesRobotRepresentation::setQ(const dVector &qNew) {
    assert(q.size() == qNew.size());
    // NOTE: we don't update the angular velocities. The assumption is that the
    // correct behavior is that the joint relative angular velocities don't
    // change, although the world relative values of the rotations do
    q = qNew;
}

// gets the current q values
void GeneralizedCoordinatesRobotRepresentation::getQ(dVector &q_copy) {
    q_copy = q;
}

void GeneralizedCoordinatesRobotRepresentation::getQFromReducedState(
    const RobotState &rs, dVector &q_copy) {
    dVector q_old = q;

    RobotState oldState(robot);
    robot->setState((RobotState *)&rs);
    syncGeneralizedCoordinatesWithRobotState();
    getQ(q_copy);
    robot->setState(&oldState);

    setQ(q_old);
}

/**
    pLocal is expressed in the coordinate frame of the link that pivots about DOF qIdx.
    This method returns the point in the coordinate frame of the parent of qIdx after
    the DOF rotation has been applied.
*/
P3D GeneralizedCoordinatesRobotRepresentation::
    getCoordsInParentQIdxFrameAfterRotation(int qIndex, const P3D &pLocal) {
    // if qIndex <= 2, this q is a component of position of the base. 
    if (qIndex <= 2) return pLocal;

    // TODO: Ex.1 Forward Kinematics
    // this is a subfunction for getWorldCoordinates() and compute_dpdq()
    // return the point in the coordinate frame of the parent of qIdx after
    // the DOF rotation has been applied.
    // 
    // Hint:
    // - use rotateVec(const V3D &v, double alpha, const V3D &axis) to get a vector 
    // rotated around axis by angle alpha.
    
    // TODO: implement your logic here.

    RBJoint *joint = getJointForQIdx(qIndex);

    V3D v_child = V3D(joint->cJPos, pLocal);

    V3D axis = joint->rotationAxis;

    double alpha = joint->getCurrentJointAngle();

    V3D v_parent = rotateVec(v_child, alpha, axis);

    P3D p_parent = joint->pJPos + v_parent;

    return p_parent;
}

// returns the world coordinates for point p, which is specified in the local
// coordinates of rb (relative to its COM): p(q)
P3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordinates(const P3D &p,
                                                                   RB *rb) {
    // TODO: Ex.1 Forward Kinematics
    // implement subfunction getCoordsInParentQIdxFrameAfterRotation() first.
    //
    // Hint: you may want to use the following functions
    // - getQIdxForJoint()
    // - getParentQIdxOf()
    // - getCoordsInParentQIdxFrameAfterRotation()


    // TODO: implement your logic here.
    //
    //

    RBJoint *qJoint = rb->pJoint;

    int qIndex = getQIdxForJoint(qJoint);

    P3D p_parent = p;

    while (qIndex>5){
        p_parent = getCoordsInParentQIdxFrameAfterRotation(qIndex, p_parent);

        qIndex = getParentQIdxOf(qIndex);
    }

    //RigidTransformation trans(robot->getHeading(), robot->root->state.pos);
    RigidTransformation trans(robot->root->state.orientation, robot->root->state.pos);

    P3D pInWorld = trans.transform(p_parent);
    
    return pInWorld;
}

// returns the global orientation associated with a specific dof q...
Quaternion GeneralizedCoordinatesRobotRepresentation::getWorldRotationForQ(
    int qIndex) {
    Quaternion qRes = Quaternion::Identity();
    // 2 here is the index of the first translational DOF of the root -- these
    // dofs do not contribute to the orientation of the rigid bodies...
    while (qIndex > 2) {
        qRes = getRelOrientationForQ(qIndex) * qRes;
        qIndex = getParentQIdxOf(qIndex);
    }
    return qRes;
}

Quaternion GeneralizedCoordinatesRobotRepresentation::getRelOrientationForQ(
    int qIndex) {
    if (qIndex < 3) return Quaternion::Identity();
    return getRotationQuaternion(q[qIndex], getQAxis(qIndex));
}

// this is a somewhat slow function to use if we must iterate through multiple
// rigid bodies...
V3D GeneralizedCoordinatesRobotRepresentation::getWorldCoordsAxisForQ(
    int qIndex) {
    if (qIndex < 3) return getQAxis(qIndex);
    return getWorldRotationForQ(qIndex) * getQAxis(qIndex);
}

// returns the world-relative orientation for rb
Quaternion GeneralizedCoordinatesRobotRepresentation::getOrientationFor(
    RB *rb) {
    int qIndex = getQIdxForJoint(rb->pJoint);
    return getWorldRotationForQ(qIndex);
}

// computes the jacobian dp/dq that tells you how the world coordinates of p
// change with q. p is expressed in the local coordinates of rb
void GeneralizedCoordinatesRobotRepresentation::compute_dpdq(const P3D &p,
                                                             RB *rb,
                                                             Matrix &dpdq) {
    resize(dpdq, 3, (int)q.size());

    
    // TODO: Ex.4 Analytic Jacobian
    //
    // Hint: you may want to use the following functions
    // - getQIdxForJoint()
    // - getCoordsInParentQIdxFrameAfterRotation()
    // - getQAxis()
    // - rotateVec()
    // - getParentQIdxOf()

    // TODO: your implementation should be here
    syncRobotStateWithGeneralizedCoordinates();
    
    dpdq.setZero();

    P3D p_world = getWorldCoordinates(p, rb);

    RBJoint *joint = rb->pJoint;

    P3D joint_child = joint->cJPos;

    RB *child = rb;

    int joint_idx = getQIdxForJoint(joint);

    while (joint_idx > 5){
        V3D n = getWorldCoordsAxisForQ(joint_idx);


        P3D joint_world = getWorldCoordinates(joint_child, child);

        V3D r_jp = V3D(joint_world, p_world);
        
        dpdq.col(joint_idx) = n.cross(r_jp);

        

        joint_idx = getParentQIdxOf(joint_idx);

        joint = getJointForQIdx(joint_idx);

        if (joint != NULL){
        joint_child = joint->cJPos;

        child = joint->child;
        }
    }

    RigidTransformation trans(robot->root->state.orientation, robot->root->state.pos);

    Matrix3x3 rotmat = trans.R.toRotationMatrix();

    P3D base_pos = trans.T;

    V3D r_bp = V3D(base_pos, p_world);

    V3D x_axis = rotmat.col(0);
    V3D y_axis = rotmat.col(1);
    V3D z_axis = rotmat.col(2);


    dpdq.col(3) = y_axis.cross(r_bp);
    dpdq.col(4) = x_axis.cross(r_bp);
    dpdq.col(5) = z_axis.cross(r_bp);
    

    dpdq.block(0, 0, 3, 3) = Matrix::Identity(3, 3);
}

// estimates the linear jacobian dp/dq using finite differences
void GeneralizedCoordinatesRobotRepresentation::estimate_linear_jacobian(
    const P3D &p, RB *rb, Matrix &dpdq) {
    resize(dpdq, 3, (int)q.size());

    for (int i = 0; i < q.size(); i++) {
        double val = q[i];
        double h = 0.0001;

        // TODO: Ex. 2-1 Inverse Kinematics - Jacobian by Finite Difference
        // compute Jacobian matrix dpdq_i by FD and fill dpdq
        q[i] = val + h;
        syncRobotStateWithGeneralizedCoordinates();
        P3D p_p = getWorldCoordinates(p, rb);  // TODO: fix this: p(qi + h);

        q[i] = val - h;
        syncRobotStateWithGeneralizedCoordinates();
        P3D p_m = getWorldCoordinates(p, rb);  // TODO: fix this: p(qi - h)
        V3D dpdq_i = V3D((p_p - p_m)/(2*h));  // TODO: fix this: compute derivative dp(q)/dqi

        // set Jacobian matrix components
        dpdq(0, i) = dpdq_i[0];
        dpdq(1, i) = dpdq_i[1];
        dpdq(2, i) = dpdq_i[2];

        // finally, we don't want to change q[i] value. back to original value.
        q[i] = val;
    }
}

}  // namespace crl