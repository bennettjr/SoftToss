#ifndef SOFTTOSS_H
#define SOFTTOSS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        ST_COLLIDER_COUNT = 32
    };

    typedef struct ST_Vec3
    {
        float x;
        float y;
        float z;
    } ST_Vec3;

    typedef struct ST_BallState
    {
        ST_Vec3 position;
        ST_Vec3 velocity;
        ST_Vec3 omega;
        ST_Vec3 acceleration;
    } ST_BallState;

    typedef struct ST_DragModel
    {
        float cd_laminar;
        float cd_turbulent;

        float Re_crit;
        float Re_width;

        float k_spin;
        float S_crit;
    } ST_DragModel;

    typedef struct ST_LiftModel
    {
        float c_l0;
        float c_l1;
        float c_l2;

        float Re_rev;
        float Re_width;

        float c_l_rev;
        float S_rev;
    } ST_LiftModel;

    typedef struct ST_CorModel
    {
        float e_n[ST_COLLIDER_COUNT];
        float k[ST_COLLIDER_COUNT];
        float beta;

        float e_t[ST_COLLIDER_COUNT];
    } ST_CorModel;

    typedef struct ST_BallSpec
    {
        float mass;
        float radius;
        float I;

        float e_n[ST_COLLIDER_COUNT];
        float e_t[ST_COLLIDER_COUNT];
        float mu_s[ST_COLLIDER_COUNT];
        float mu_k[ST_COLLIDER_COUNT];
        float c_rr[ST_COLLIDER_COUNT];

        ST_DragModel dragModel;
        ST_LiftModel liftModel;
        ST_CorModel corModel;
    } ST_BallSpec;

    typedef struct ST_Environment
    {
        float gravity;
        float temp;
        float elev;
        float humid;
        float pres;
        float mu;
        float rho;
        float windSpeed;
        float windDir;
        float windHeight;
    } ST_Environment;

    typedef struct ST_Collider
    {
        uint8_t type;
        ST_Vec3 point;
        ST_Vec3 velocity;
        ST_Vec3 omega;
        ST_Vec3 leverArm;
        float invMass;
        float invI_0;
        float invI_z;
    } ST_Collider;

    typedef enum ST_Integrator
    {
        ST_Integrator_Euler = 0,
        ST_Integrator_RK4 = 1
    } ST_Integrator;

    void SoftToss_updateState(const ST_BallSpec *spec, const ST_BallState *state, const ST_Environment *env, float dt, ST_Integrator integrator, const ST_Collider *collider, ST_BallState *out);

#ifdef __cplusplus
}
#endif

#endif