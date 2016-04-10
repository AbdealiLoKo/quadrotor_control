# Create a trajectory using a set of observed trajectories

import numpy as np
import scipy as sp
import sympy as sym
from scipy.spatial.distance import euclidean

from dtw import dtw
from pykalman import UnscentedKalmanFilter


n_dim_state = 8
n_dim_action = 4
n_dim_kalman_state = n_dim_state + n_dim_action
n_dim_kalman_observed = n_dim_kalman_state

observed_trajectories = np.array([
np.matrix("""
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    1.0, 0.0, 0.0, 0.0;
2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
3.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0;
4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0
"""),
np.matrix("""
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
1.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    1.0, 0.0, 0.0, 0.0;
2.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
3.2, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0;
4.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0
"""),
np.matrix("""
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
1.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    1.0, 0.0, 0.0, 0.0;
2.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,    0.5, 0.0, 0.0, 0.0;
2.9, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0;
4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    0.0, 0.0, 0.0, 0.0
"""),
]) # i.shape = time_steps, n_dim_kalman_observed
for i in observed_trajectories:
    assert i.shape[1] == n_dim_kalman_observed

num_observed_trajectories = len(observed_trajectories)
observed_time_length = [i.shape[0] for i in observed_trajectories]

hidden_time_length = int(2.0 * np.mean(observed_time_length))
hidden_trajectory = np.zeros([hidden_time_length, n_dim_kalman_state])

def get_time_warped_series(series1, series2=hidden_trajectory):
    """
    Takes 2 series, s1, s2. Applies dynamic time warping to both the
    trajectories.
    Returns the warped series with length of series2 but corresponding values
    in series1.
    """
    assert len(series1) <= len(series2)

    dist, cost, acc, path = dtw(series1, series2, euclidean)
    p1, p2 = path
    # Warp the larger series to the smaller one
    # new_mapping = np.array([
    #     np.mean(series2[p1 == i,:], axis=0) for i in sorted(set(p1)) ])
    # assert len(series1) == len(new_mapping)

    # Warp the smaller series to the larger one
    assert len(series2) == len(p1)
    new_mapping = series1[p1,:]
    assert len(series2) == len(new_mapping)

    return new_mapping

# time_mapping = np.array([get_time_warped_series(i) for i in observed_trajectories])

print("Number of trajectories: {}".format(len(observed_trajectories)))
print("Length of trajectories: {}".format([len(i) for i in observed_trajectories]))
print("Length of hidden trajectory: {}".format(hidden_time_length)

def _observation(current_kalman_state, noise):
                 # observed_trajectories=observed_trajectories,
                 # time_mapping):
    """
    Gets the current (hidden) state and noise for an observation at this time.
    Gives out the probable observed state based on this noise and other
    data at the same time instance.
    time_mapping is the time mapped hidden state of the kalman state.

    First use the time mapping to identify which time instance this state is
    related to and then use the hidden state for that to get the observed
    value.
    """
    print("In _observation: ", current_kalman_state, noise)
    # Simple gaussian noise added
    return current_kalman_state + noise

x,y,z,w,vx,vy,vz,vw,ax,ay,az,aw = sym.symbols('x y z w vx vy vz vw ax ay az aw')
alph = 0.5
system_dyamics = sym.Matrix([
    x + vx + alph*(ax-vx),
    y + vy + alph*(ay-vy),
    z + vz + alph*(az-vz),
    w + vw + alph*(aw-vw),
    vx + alph*(ax-vx),
    vy + alph*(ay-vy),
    vz + alph*(az-vz),
    vw + alph*(aw-vw),
    ax,
    ay,
    az,
    aw
])
state = sym.Matrix([x,y,z,w,vx,vy,vz,vw,ax,ay,az,aw])
jacobian_dynamics = system_dyamics.jacobian(state)

def _transition(current_kalman_state, noise):
                #observed_trajectories=observed_trajectories,
                #time_mapping=time_mapping):
    """
    Gets the current state and noise at this transition.
    Gives out the next state based on this data.

    This function depends on the model of the quadrotor alone and not on
    anything else. It is used in the Kalman filter for the hidden states to
    identify the next state.
    """
    print("In _transition: ", current_kalman_state, noise)
    # current_kalman_state[:n_dim_state, :]
    next_state = system_dyamics.subs(list(zip(state, current_kalman_state)))
    return list(next_state)

def jacobian_transition(current_kalman_state, noise):
    j = jacobian.subs(list(zip(state, current_kalman_state)))
    return list(j)

ukf = UnscentedKalmanFilter(
    transition_functions=_transition,
    observation_functions=_observation,
    # transition_covariance=None,
    # observation_covariance=None,
    # initial_state_mean=None,
    # initial_state_covariance=None,
    n_dim_state=n_dim_kalman_state,
    n_dim_obs=n_dim_kalman_observed,
    random_state=None)

n = 100
while n:
    n -= 1

    # Find the means and covariances

    for obs in observed_trajectories:
        # warp trajectory to make it to the same length of hidden trajectory
        warped_obs = get_time_warped_series(obs)
        smooth_means, smooth_cov = ukf.smooth(warped_obs)

    # Find new Q and R
    smooth_means
    smooth_cov
    filter_means
    fliter_cov
    Q = ukf.transition_covariance
    R = ukf.observation_covariance

    new_Q = 0
    new_R = 0
    for t in range(hidden_time_length-1):
        d_mu = smooth_means[t+1] - _transition(smooth_means[t])
        A = jacobian_transition(smooth_means[i])
        L = filter_cov[t] * A.T / filter_cov[t+1]
        P = smooth_cov[t+1] - smooth_cov[t+1] * L.T * A.T - A.T * L.T * smooth_cov[t+1]
        new_Q += d_mu * d_mu.T + A * smooth_cov[t] * A.T + P

        d_y = warped_obs[t] - _observation(smooth_means[t]) # TODOOOO
        C = jacobian_transition(smooth_means[t])
        new_R += d_y * d_y.T + C * filter_cov[t] * C.T
    new_Q = new_Q / (hidden_time_length - 1)
    new_R = new_R / (hidden_time_length - 1)

    ukf.transition_covariance = new_Q
    ukf.observation_covariance = new_R
