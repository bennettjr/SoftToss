#ifndef SOFTTOSS_H
#define SOFTTOSS_H

#ifdef __cplusplus
extern "C"
{
#endif

    // C-friendly API types (prefixed with ST_ to avoid colliding with C++ types)
    typedef struct ST_Vec3
    {
        float x, y, z;
    } ST_Vec3;

    typedef struct ST_BallState
    {
        ST_Vec3 position;
        ST_Vec3 velocity;
        ST_Vec3 omega;
        ST_Vec3 acceleration;
    } ST_BallState;

    typedef struct ST_BallSpec
    {
        float mass;
        float radius;
        float I;

        float c_d0;
        float c_d;
        float c_l0;
        float c_l1;
        float c_l2;
    } ST_BallSpec;

    typedef struct ST_Environment
    {
        float temp;
        float elev;
        float humid;
        float pres;

        float rho;

        float windSpeed;
        float windDir;
        float windHeight;
    } ST_Environment;

    // Collider type mirrors SoftToss::ColliderType
    typedef enum ST_ColliderType
    {
        ST_Collider_Dirt = 0,
        ST_Collider_Grass = 1,
    } ST_ColliderType;

    typedef struct ST_Collider
    {
        ST_ColliderType type;
        ST_Vec3 point;
    } ST_Collider;

    // Update the ball state. Provide pointers to inputs; `collider` may be NULL.
    // The result is written into `out`.
    void SoftToss_updateState(const ST_BallSpec *spec, const ST_BallState *state, const ST_Environment *env, float dt, const ST_Collider *collider, ST_BallState *out);

#ifdef __cplusplus
}
#endif

#endif