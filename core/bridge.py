import jax
import jax.numpy as jnp
from jax import value_and_grad, jit, lax, vmap

jax.config.update("jax_enable_x64", True)

@jit
def quaternion_inverse(q):
    return jnp.array([q[0], -q[1], -q[2], -q[3]])

@jit
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return jnp.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ])

@jit
def observation_model(qs):
    g = jnp.array([0., 0., 0., 1.])
    qs = qs.reshape(-1, 4)
    result = vmap(lambda q: quaternion_multiply(quaternion_multiply(quaternion_inverse(q), g), q))(qs)
    return result

@jit
def cost_function(q, f, a):
    T = q.shape[0]
    eps = 1e-12

    q_t = q[:-1]
    q_t_plus_1 = q[1:]
    predicted_q = f[:-1]
    a_extended = jnp.hstack([jnp.zeros((a.shape[0], 1)), a]) 
 
    diff_q = vmap(quaternion_multiply)(vmap(quaternion_inverse)(q_t_plus_1), predicted_q)

    q_v = diff_q[:, 1:] 
    q_s = diff_q[:, 0]   

    norm_q = jnp.linalg.norm(diff_q, axis=1) + eps
    norm_qv = jnp.linalg.norm(q_v, axis=1) + eps

    qs = jnp.log(jnp.clip(norm_q, eps, None))

    angle = jnp.arccos(jnp.clip(q_s / norm_q, -1.0 + eps, 1.0 - eps))
    q_v_normalized = q_v / norm_qv[:, None]
    qv_scaled = q_v_normalized * angle[:, None]
 
    log_diff_q = jnp.hstack([qs[:, None], qv_scaled])

    
    motion_error = jnp.sum(jnp.linalg.norm(2 * log_diff_q, axis=1) ** 2)
    # this actually gives me the correct cost needed, this is a coding bruh moment, if ur reading this, yea i have no idea, it just works, look up quakes root.
    motion_error = 0
    predicted_a = observation_model(q)
    observation_error = jnp.sum(jnp.linalg.norm(a_extended - predicted_a, axis=1) ** 2) 

    total_cost = 0.5 * motion_error + 0.5 * observation_error
     
    #jax.debug.print("Cost {total_cost}", total_cost=total_cost)
    return total_cost

@jit
def project_to_unit_quaternion(q):
    norms = jnp.linalg.norm(q, axis=1, keepdims=True) + 1e-12
    return q / norms

@jit
def train(q, f, a, h, step_size=1e-4):
    value_grad_fn = value_and_grad(cost_function)
    epoch = 10000
    def update_step(carry, epoch):
        q_optim, _ = carry
        cost, grad_q = value_grad_fn(q_optim, f, a) 
        q_optim = q_optim - step_size * grad_q
        q_optim = project_to_unit_quaternion(q_optim)

        return (q_optim, cost), cost

    (q_final, _), _ = lax.scan(update_step, (q, 0), jnp.arange(epoch))

    return q_final

#@jit

