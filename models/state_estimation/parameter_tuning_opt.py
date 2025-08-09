import numpy as np
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from uav_model import run_scenario
from scipy.spatial.transform import Rotation as R

# Define the search space for the parameters
param_space = [
    Real(1e-4, 1e-1, name='process_noise_pos', prior='log-uniform'),
    Real(1e-3, 1e+0, name='process_noise_vel', prior='log-uniform'),
    Real(1e-4, 1e-1, name='process_noise_quat', prior='log-uniform'),
    Real(1e-3, 1e+0, name='process_noise_omega', prior='log-uniform'),

    Real(1e+0, 1e+2, name='pos_uncertainty', prior='log-uniform'),
    Real(1e-1, 1e+1, name='vel_uncertainty', prior='log-uniform'),
    Real(1e-2, 1e+0, name='quat_uncertainty', prior='log-uniform'),
    Real(1e-2, 1e+0, name='omega_uncertainty', prior='log-uniform')
]

# Reference scenario to optimize against
base_scenario = {
    "name": "Straight and Level",
    "config": {
        "name": "custom",
        "start_orientation": np.array([0, 0, 0, 1]),
        "end_orientation": np.array([0, 0, 0, 1]),
        "start_velocity": np.array([0.0, 10.0, 0.0]),
        "end_velocity": np.array([0.0, 10.0, 0.0]),
        "start_position": np.array([0.0, 0.0, 0.0]),
        "end_position": np.array([0.0, 1000.0, 0.0]),
        "duration_sec": 100.0,
        "speed": 10.0,
        "waypoints": None
    }
}

# Scoring function
@use_named_args(param_space)
def objective(**params):
    config = base_scenario['config'].copy()
    config['tuning_params'] = params

    metrics = run_scenario(config, plot=False)

    # Initial convergence metric
    early_pos_err = np.linalg.norm(metrics['pos_err'][:10], axis=1).mean()
    early_vel_err = np.linalg.norm(metrics['vel_err'][:10], axis=1).mean()

    score = (
        metrics['rmse_pos'].mean() * 1.0 +
        metrics['rmse_vel'].mean() * 1.0 +
        metrics['rmse_orientation'] * 1.0 +
        abs(1 - metrics['pos_nes']) * 2.0 +
        abs(1 - metrics['vel_nes']) * 2.0 +
        early_pos_err * 0.5 +
        early_vel_err * 0.5
    )
    print(f"Score: {score:.3f} | Params: {params}")
    return score

if __name__ == "__main__":
    print("Starting Bayesian Optimization with scikit-optimize...")

    res = gp_minimize(
        func=objective,
        dimensions=param_space,
        n_calls=100,
        n_random_starts=25,
        random_state=42,
        verbose=True
    )

    best_params = dict(zip([dim.name for dim in param_space], res.x))
    print("\nBest Parameters:")
    for k, v in best_params.items():
        print(f"{k}: {v:.6f}")

    with open("best_tuned_params.txt", "w") as f:
        for k, v in best_params.items():
            f.write(f"{k}: {v:.6f}\n")
