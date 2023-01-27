#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

using Eigen::MatrixXd;
template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename S, int a>
using VectorBis = Eigen::Matrix<S, a, 1, 0, a, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum MethodTypes {
        FREEFALL=0, SEPARATION=1, ALIGNMENT=2, COHESION=3, LEADER=4, CIRCULAR=5, COLLISIONAVOIDANCE=6, PREDATORPRAY=7
    };

enum IntegrationMethods {
        EXPLICIT=0, SYMPLETIC=1, MIDPOINT=2
    };



using T = double;
using VectorXT = Matrix<T, Eigen::Dynamic, 1>;
using TV = Vector<T, 2>;
using TM = Matrix<T, 2, 2>;

using S = std::chrono::high_resolution_clock::time_point;
using VectorSec = VectorBis<S, Eigen::Dynamic>;

class Boids
{
    
    int dim = 2;
    
private:
    VectorXT positions;
    VectorXT velocities;
    VectorXT group;
    VectorSec times;
    int n;
    bool update = false;

public:
    Boids() :n(1) {}
    Boids(int n, MethodTypes type) :n(n) {
        initializePositions(type);
    }
    ~Boids() {}

    void setParticleNumber(int n) {n = n;}
    int getParticleNumber() { return n; }
    void initializePositions(MethodTypes type)
    {
        positions = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);}); 
        velocities = VectorXT::Zero(n * dim);

        if (type == 0 || type == 1 || type == 2 || type == 6 || type == 7){
            velocities = VectorXT::Zero(n * dim).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);});
        }

        if (type == 5){
            for (int i=0; i<velocities.rows(); i++){
                if (i%2==0){
                    velocities(i) = -positions(i+1)+0.5;
                } else{
                    velocities(i) = positions(i-1)-0.5;
                }
            }
        }

        group = VectorXT::Zero(n);
        group.tail(n/2) = VectorXT::Ones(n/2);


        VectorBis<S, 40> temp;

        for (int i=0; i<temp.rows(); i++){
            temp(i) = std::chrono::high_resolution_clock::now();
        }
        
        times = temp;
    }

    void updateBehavior(MethodTypes type, IntegrationMethods intMet, double h, VectorXT mouse_pos)
    {
        MatrixXd force_mat;

        if(!update)
            return;
        switch (intMet)
        {  
        case(0):
            explicitEuler(h, type, mouse_pos);
            break;

        case(1):
            sympleticEuler(h, type, mouse_pos);
            break;

        case(2):
            explicitMidpoint(h, type, mouse_pos);
            break;

        default:
            break;
        }
    }
    void pause()
    {
        update = !update;
    }


    void explicitEuler(double &h, MethodTypes type, VectorXT mouse_pos){
        
        MatrixXd accel;
        MatrixXd force_mat = getForce(type, mouse_pos);
        accel = force_mat;
        
        accel.transposeInPlace();
        accel.resize(velocities.rows(), 1); //pos and vel are vectors (x1, y1, x2, y2, ..., xn, yn)T

        positions = positions + h*velocities;
        velocities = velocities + h*accel;

        limit_vel();
    } 


    void sympleticEuler(double &h, MethodTypes type, VectorXT mouse_pos){
        positions = positions + h*velocities;

        MatrixXd accel;
        MatrixXd force_mat = getForce(type, mouse_pos);
        accel = force_mat;
        
        accel.transposeInPlace();
        accel.resize(velocities.rows(), 1);        
        
        velocities = velocities + h*accel;

        limit_vel();
    }


    void explicitMidpoint(double &h, MethodTypes type, VectorXT mouse_pos){
        VectorXT old_pos = positions;
        VectorXT old_vel = velocities;

        MatrixXd accel;
        MatrixXd force_mat = getForce(type, mouse_pos);
        accel = force_mat;
        
        accel.transposeInPlace();
        accel.resize(velocities.rows(), 1); //pos and vel are vectors (x1, y1, x2, y2, ..., xn, yn)T

        positions = positions + h*velocities/2;
        velocities = velocities + h*accel/2;
        


        force_mat = getForce(type, mouse_pos);
        accel = force_mat;
        
        accel.transposeInPlace();
        accel.resize(velocities.rows(), 1); //pos and vel are vectors (x1, y1, x2, y2, ..., xn, yn)T

        positions = old_pos + h*velocities;
        velocities = old_vel + h*accel;      

        //limit_vel();  
    }
    
    void limit_vel(){
        MatrixXd vel_reshaped = velocities;
        MatrixXd vel_magnitude;
        double threshold = 1.6;
        
        vel_reshaped.resize(2, positions.rows()/2);

        vel_magnitude = vel_reshaped.colwise().norm();

        for (int i=0; i<vel_reshaped.cols(); i++){
            if (vel_magnitude(i) > threshold){
                velocities(2*i) = velocities(2*i)/vel_magnitude(i)*threshold;
                velocities(2*i+1) = velocities(2*i+1)/vel_magnitude(i)*threshold;
            }
        }
    }


    MatrixXd getForce(MethodTypes type, VectorXT mouse_pos)
    {
        switch (type)
        {  
        case(0):
            return weight_force();
            break;

        case(1):
            return repulsive_force();
            break;

        case(2):
            return alignment_force();
            break;

        case(3):
            return cohesion_force();
            break;
        
        case(4):
            return leading_force(mouse_pos);
            break;

        case(5):
            return centripetal_force();
            break;

        case(6):
            return collisionAvoidance_force();
            break;

        case(7):
            return predpray_force();
            break;

        default:
            break;
        }
    }


    MatrixXd weight_force(){
        MatrixXd acc(velocities.rows()/2, 2);
        acc.col(0) = VectorXT::Zero((acc.rows(), 1));
        acc.col(1) = 9.81*VectorXT::Ones((acc.rows(), 1)); //y points downward

        MatrixXd fp = acc;
        return fp;
    }   


    MatrixXd repulsive_force(){

        VectorXT distances(1, positions.rows()/2);
        MatrixXd rep_accel = MatrixXd::Zero(2, positions.rows()/2);
        MatrixXd rep_forces;
        MatrixXd pos_reshaped = positions;

        pos_reshaped.resize(2, positions.rows()/2);
        
        for (int i=0; i<rep_accel.cols(); i++){
            distances = (pos_reshaped.colwise() - pos_reshaped.col(i)).colwise().norm();
            distances(i) = 1;
            for (int j=0; j<rep_accel.cols(); j++){
                if (distances(j)<0.05){
                    rep_accel.col(i) -= 2*(pos_reshaped.col(j) - pos_reshaped.col(i))/distances(j);
                }
            }            
        }        
        rep_forces = rep_accel.transpose();

        return rep_forces;
    }


    MatrixXd alignment_force(){

        VectorXT distances(1, positions.rows()/2);
        VectorXT aver(2, 1);
        MatrixXd average_vel(2, positions.rows()/2);
        MatrixXd al_forces;
        MatrixXd pos_reshaped = positions;
        MatrixXd vel_reshaped = velocities;
        int n_neig;

        pos_reshaped.resize(2, positions.rows()/2);
        vel_reshaped.resize(2, positions.rows()/2);

        average_vel = vel_reshaped;
        
        for (int i=0; i<average_vel.cols(); i++){
            distances = (pos_reshaped.colwise() - pos_reshaped.col(i)).colwise().norm();
            distances(i) = 1;
            n_neig = 0;
            aver = VectorXT::Zero(2, 1);
            for (int j=0; j<average_vel.cols(); j++){
                if (distances(j)<0.2){
                    n_neig++;
                    aver += vel_reshaped.col(j);
                }
            }
            if (n_neig>0){
                average_vel.col(i) = aver/n_neig;
            }
            
        }        

        al_forces = 6 * (average_vel - vel_reshaped).transpose();

        return al_forces;
    }


    MatrixXd cohesion_force(){

        VectorXT distances(1, positions.rows()/2);
        VectorXT aver(2, 1);
        MatrixXd average_pos(2, positions.rows()/2);
        MatrixXd coh_forces;
        MatrixXd coh_accel(2, positions.rows()/2);
        MatrixXd pos_reshaped = positions;
        MatrixXd vel_reshaped = velocities;
        int n_neig;

        pos_reshaped.resize(2, positions.rows()/2);
        vel_reshaped.resize(2, positions.rows()/2);

        average_pos = pos_reshaped;
        
        for (int i=0; i<average_pos.cols(); i++){
            distances = (pos_reshaped.colwise() - pos_reshaped.col(i)).colwise().norm();
            distances(i) = 1;
            n_neig = 0;
            aver = VectorXT::Zero(2, 1);
            for (int j=0; j<average_pos.cols(); j++){
                if (distances(j)<0.5){
                    n_neig++;
                    aver += pos_reshaped.col(j);
                }
            }
            if (n_neig>0){
                average_pos.col(i) = aver/n_neig;
            }

            coh_accel.col(i) = (average_pos.col(i) - pos_reshaped.col(i))/(average_pos.col(i) - pos_reshaped.col(i)).norm();
            
        }        

        coh_forces = 5 * coh_accel.transpose();

        return coh_forces;
    }


    MatrixXd leading_force(VectorXT mouse_pos){
        VectorXT leader_distance(1, positions.rows()/2);
        VectorXT distances(1, positions.rows()/2);
        MatrixXd lead_accel = MatrixXd::Zero(2, positions.rows()/2);
        MatrixXd lead_forces;
        MatrixXd pos_reshaped = positions;
        MatrixXd vel_reshaped = velocities;
        MatrixXd ca_forces = collisionAvoidance_force();
        MatrixXd rep_forces = repulsive_force();

        pos_reshaped.resize(2, positions.rows()/2);
        vel_reshaped.resize(2, positions.rows()/2);

        leader_distance = (pos_reshaped.colwise() - pos_reshaped.col(0)).colwise().norm();

        for (int i=1; i<lead_accel.cols(); i++){
            distances = (pos_reshaped.colwise() - pos_reshaped.col(i)).colwise().norm();
            distances(i) = 1;
            for (int j=1; j<lead_accel.cols(); j++){
                if (distances(j)<0.1){
                    lead_accel.col(i) -= 0.1*(pos_reshaped.col(j) - pos_reshaped.col(i))/distances(j);
                }
            }

            if (leader_distance(i)<0.05){
                    lead_accel.col(i) -= 10*(pos_reshaped.col(0) - pos_reshaped.col(i))/leader_distance(i);
            }
            
            if (leader_distance(i)<0.7){
                lead_accel.col(i) += 8*(pos_reshaped.col(0) - pos_reshaped.col(i));
                lead_accel.col(i) += 2*(vel_reshaped.col(0) - vel_reshaped.col(i));
            }
        }
        

        lead_accel.col(0) = 10*(mouse_pos - pos_reshaped.col(0));
        lead_accel.col(0) -= 1*vel_reshaped.col(0);
        
        if ((mouse_pos - pos_reshaped.col(0)).norm() > 0.001){
            lead_accel.col(0)/=(mouse_pos - pos_reshaped.col(0)).norm();
        }

        lead_forces = lead_accel.transpose() + 0.5*ca_forces + rep_forces;
        
        return lead_forces;
    }


    MatrixXd centripetal_force(){
        VectorXT dist_squared(positions.rows()/2, 1);
        for (int i=0; i<dist_squared.rows(); i++){
            dist_squared(i) = pow(positions(i*2)-0.5, 2) + pow(positions(i*2+1)-0.5, 2);
        }

        MatrixXd c_acc(velocities.rows()/2, 2);
        for (int i=0; i<c_acc.rows(); i++){

            c_acc(i, 0) = -(positions(i*2)-0.5);
            c_acc(i, 1) = -(positions(i*2+1)-0.5);
        }

        MatrixXd c_force = c_acc;
        return c_force;
    }


    MatrixXd collisionAvoidance_force(){
        VectorXT distances(1, positions.rows()/2);
        MatrixXd ca_accel = MatrixXd::Zero(2, positions.rows()/2);
        MatrixXd ca_forces;
        MatrixXd pos_reshaped = positions;
        MatrixXd vel_reshaped = velocities;

        VectorXT next_dist(1, positions.rows()/2);

        VectorXT Center(2, 1);
        Center << 0.7, 0.7;
        double ray = 0.2;

        pos_reshaped.resize(2, positions.rows()/2);
        vel_reshaped.resize(2, positions.rows()/2);

        distances = (pos_reshaped.colwise() - Center).colwise().norm();
        next_dist = ((pos_reshaped + 0.001*vel_reshaped).colwise() - Center).colwise().norm();

        for (int i=0; i<ca_accel.cols(); i++){
            if (distances(i) <= ray && distances(i)>next_dist(i)){
                ca_accel.col(i) = -600*vel_reshaped.col(i);
            }
        }

        ca_forces = ca_accel.transpose();

        return ca_forces;
    }

    MatrixXd predpray_force(){
        MatrixXd distances(positions.rows()/2, positions.rows()/2);
        MatrixXd pp_accel = MatrixXd::Zero(2, positions.rows()/2);
        MatrixXd pp_forces;
        MatrixXd pos_reshaped = positions;
        MatrixXd vel_reshaped = velocities;        

        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

        pos_reshaped.resize(2, positions.rows()/2);
        vel_reshaped.resize(2, positions.rows()/2);

        int count_enemies;
        int count_friends;
        double index;
        
        for (int i=0; i<distances.rows(); i++)
        {
            distances.row(i) = (pos_reshaped.colwise() - pos_reshaped.col(i)).colwise().norm();
        }

        for (int i=0; i<distances.rows(); i++){
            count_enemies = 0;
            count_friends = 0;
            VectorXT enemies(0);
            VectorXT friends(0);
            VectorXT dist_friends(0);

            auto old_i = times(i);
            auto diff_i = std::chrono::duration_cast<std::chrono::microseconds>(now-old_i).count();

            for(int j=0; j<i; j++){
                auto old_j = times(j);
                auto diff_j = std::chrono::duration_cast<std::chrono::microseconds>(now-old_j).count();

                if (distances(i, j)<=0.01 && group(i)==group(j) && diff_i >= 10.*10./60. * 1.e6 && diff_j >= 10.*10./60. * 1.e6){
                    add_boid(group(i));
                    pp_accel.conservativeResize(2, pp_accel.cols()+1);
                    pp_accel.col(pp_accel.cols()-1) = VectorXT::Zero(2, 1);

                    times(i) = now;
                    times(j) = now;
                }
            }
            
            for(int j=0; j<distances.cols(); j++){
                if (distances(i, j)<0.08 && group(i)!=group(j) && i!=j){
                    count_enemies++;
                }

                if (distances(i, j)<0.08 && group(i)==group(j) && i!=j){
                    count_friends++;
                }

                if (distances(i, j)<=0.13 && group(i)!=group(j) && i!=j){
                    enemies.conservativeResize(enemies.size()+1);
                    enemies(enemies.size()-1) = j;
                }
                if (distances(i, j)<=0.13 && group(i)==group(j) && i!=j){
                    friends.conservativeResize(friends.size()+1);
                    friends(friends.size()-1) = j;
                }
                if (distances(i, j)<=0.2 && group(i)==group(j) && i!=j){
                    dist_friends.conservativeResize(dist_friends.size()+1);
                    dist_friends(dist_friends.size()-1) = j;
                }
            }
            
            
            if ((count_enemies - count_friends)>=3){
                remove_boid(i);
            }


            if ((enemies.size() - friends.size())>=3){
                for (int j=0; j<enemies.size(); j++){
                    index = enemies(j);
                    pp_accel.col(i) -= 13*(pos_reshaped.col(index) - pos_reshaped.col(i))/distances(i, index);
                }
            }

            if ((friends.size() - enemies.size())>=2){
                for (int j=0; j<enemies.size(); j++){
                    index = enemies(j);
                    pp_accel.col(i) += 13*(pos_reshaped.col(index) - pos_reshaped.col(i))/distances(i, index);
                }
            }

            for (int j=0; j<dist_friends.size(); j++){
                index = dist_friends(j);
                pp_accel.col(i) += 5*(pos_reshaped.col(index) - pos_reshaped.col(i))/distances(i, index);
            }
        }
        

        pp_forces = pp_accel.transpose();
        return pp_forces;
    }

    void add_boid(double group_type){
        positions.conservativeResize(positions.size()+2);
        positions(positions.size()-2) = 1.3 * VectorXT::Zero(1).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);})(0); 
        positions(positions.size()-1) = 1.3 * VectorXT::Zero(1).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);})(0); 

        velocities.conservativeResize(velocities.size()+2);
        velocities(velocities.size()-2) = VectorXT::Zero(1).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);})(0); 
        velocities(velocities.size()-1) = VectorXT::Zero(1).unaryExpr([&](T dummy){return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);})(0); 

        group.conservativeResize(group.size()+1);
        group(group.size()-1) = group_type;

        times.conservativeResize(times.size()+1);
        times(times.size()-1) = std::chrono::high_resolution_clock::now();
    }

    void remove_boid(double index){
        positions(2*index)=std::numeric_limits<double>::quiet_NaN();
        positions(2*index+1)=std::numeric_limits<double>::quiet_NaN();
        velocities(2*index)=std::numeric_limits<double>::quiet_NaN();
        velocities(2*index+1)=std::numeric_limits<double>::quiet_NaN();
        group(index)=std::numeric_limits<double>::quiet_NaN();
    }


    VectorXT getPositions(){
        return positions;
    }

    VectorXT getGroup(){
        return group;
    }
};
#endif
