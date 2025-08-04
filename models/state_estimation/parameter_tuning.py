import numpy as np
from itertools import product
from uav_model import run_scenario
from scipy.spatial.transform import Rotation as R

class ParameterTuner:
  def __init__(self):
    # Parameter ranges based on physical uncertainties
    self.param_ranges = {
        # Process noise parameters (per second)
        'process_noise_pos': np.logspace(-4, -1, 8),    # 0.1mm to 10cm position drift
        'process_noise_vel': np.logspace(-3, 0, 8),     # 1mm/s to 1m/s velocity drift
        'process_noise_acc': np.logspace(-2, 1, 8),     # 0.01 to 10 m/s² acceleration noise
        'process_noise_quat': np.logspace(-4, -1, 8),   # 0.0001 to 0.1 rad orientation drift
        'process_noise_omega': np.logspace(-3, 0, 8),   # 0.001 to 1 rad/s angular rate drift

        # Initial uncertainty parameters
        'pos_uncertainty': np.logspace(0, 2, 8),        # 1m to 100m initial position uncertainty
        'vel_uncertainty': np.logspace(-1, 1, 8),       # 0.1 to 10 m/s initial velocity uncertainty
        'acc_uncertainty': np.logspace(-1, 1, 8),       # 0.1 to 10 m/s² initial acceleration uncertainty
        'quat_uncertainty': np.logspace(-2, 0, 8),      # 0.01 to 1 rad initial orientation uncertainty
        'omega_uncertainty': np.logspace(-2, 0, 8)      # 0.01 to 1 rad/s initial angular rate uncertainty
    }
    
    self.best_params = None
    self.best_score = float('inf')
      
  def evaluate_params(self, params, scenario_config):
    """Run scenario with given parameters and return performance score"""
    config = scenario_config.copy()
    config['tuning_params'] = params
    
    metrics = run_scenario(config, plot=False)
    
    # Compute early convergence error (first second)
    n_early_samples = 10  # Assuming 10Hz data rate
    early_pos_err = np.linalg.norm(metrics['pos_err'][:n_early_samples], axis=1).mean()
    early_vel_err = np.linalg.norm(metrics['vel_err'][:n_early_samples], axis=1).mean()
    
    # Comprehensive scoring
    score = (
        metrics['rmse_pos'].mean() * 1.0 +         # Overall position accuracy
        metrics['rmse_vel'].mean() * 1.0 +         # Overall velocity accuracy
        metrics['rmse_orientation'] * 1.0 +        # Overall orientation accuracy
        abs(1 - metrics['pos_nes']) * 2.0 +        # Position estimate consistency
        abs(1 - metrics['vel_nes']) * 2.0 +        # Velocity estimate consistency
        early_pos_err * 0.5 +                      # Initial position convergence
        early_vel_err * 0.5                        # Initial velocity convergence
    )
    
    return score, metrics
      
  def run_grid_search(self, scenario_config, n_samples=100):
    """Run random grid search over parameter space"""
    print(f"\nStarting parameter search for scenario: {scenario_config['name']}")
    print(f"Running {n_samples} iterations...")
    results = []
    
    for iteration in range(n_samples):
      params = {
          key: np.random.choice(values)
          for key, values in self.param_ranges.items()
      }
      
      score, metrics = self.evaluate_params(params, scenario_config)
      results.append((score, params, metrics))
        
      # Update best parameters
      if score < self.best_score:
        self.best_score = score
        self.best_params = params
        print(f"\nIteration {iteration}: New best score: {score:.3f}")
        print("\nParameters:")
        for k, v in params.items():
          print(f"{k}: {v:.6f}")
        print("\nMetrics:")
        print(f"Position RMSE: {metrics['rmse_pos']}")
        print(f"Velocity RMSE: {metrics['rmse_vel']}")
        print(f"Orientation RMSE: {metrics['rmse_orientation']:.2f}°")
        print(f"Position NES: {metrics['pos_nes']:.3f}")
        print(f"Velocity NES: {metrics['vel_nes']:.3f}")
            
    return sorted(results, key=lambda x: x[0])

if __name__ == "__main__":
  
  # Define test scenarios
  straight_scenario = {
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

  circle_scenario = {
      "name": "circle",
      "config": {
          "name": "circle",
          "start_orientation": np.array([0, 0, 0, 1]),
          "end_orientation": np.array([0, 0, 0, 1]),
          "start_velocity": np.array([0.0, 0.0, 0.0]),
          "end_velocity": np.array([0.0, 0.0, 0.0]),
          "start_position": np.array([100.0, 0.0, 100.0]),
          "end_position": np.array([100.0, 0.0, 100.0]),
          "duration_sec": 60.0,
          "speed": 10.0,
          "waypoints": None
      }
  }
  
  climbing_scenario = {
      "name": "climbing",
      "config": {
          "name": "custom",
          "start_orientation": np.array([0, 0, 0, 1]),
          "end_orientation": R.from_euler('xyz', [0, 15, 0], degrees=True).as_quat(),
          "start_velocity": np.array([10.0, 0.0, 0.0]),
          "end_velocity": np.array([9.659, 0.0, 2.588]),
          "start_position": np.array([0.0, 0.0, 100.0]),
          "end_position": np.array([500.0, 0.0, 250.0]),
          "duration_sec": 50.0,
          "speed": 10.0,
          "waypoints": None
      }
  }
  
  # Create tuner instance
  tuner = ParameterTuner()
  
  # Don't override the parameter ranges, they're already defined in __init__
  # Instead, can adjust individual values if needed
  tuner.param_ranges.update({
      'process_noise_pos': np.logspace(-4, -1, 8),    # 0.1mm to 10cm position drift
      'process_noise_vel': np.logspace(-3, 0, 8),     # 1mm/s to 1m/s velocity drift
      'process_noise_acc': np.logspace(-2, 1, 8),     # 0.01 to 10 m/s^2 acceleration noise
      'process_noise_quat': np.logspace(-4, -1, 8),   # 0.0001 to 0.1 rad orientation drift
      'process_noise_omega': np.logspace(-3, 0, 8),   # 0.001 to 1 rad/s angular rate drift
      'pos_uncertainty': np.logspace(0, 2, 8),        # 1m to 100m initial position uncertainty
      'vel_uncertainty': np.logspace(-1, 1, 8),       # 0.1 to 10 m/s initial velocity uncertainty
      'acc_uncertainty': np.logspace(-1, 1, 8),       # 0.1 to 10 m/s^2 initial acceleration uncertainty
      'quat_uncertainty': np.logspace(-2, 0, 8),      # 0.01 to 1 rad initial orientation uncertainty
      'omega_uncertainty': np.logspace(-2, 0, 8)      # 0.01 to 1 rad/s initial angular rate uncertainty
  })
  
  # Run tuning for each scenario
  n_samples = 1000  # Increased for better coverage
  scenarios = [straight_scenario, circle_scenario, climbing_scenario]
  
  for scenario in scenarios:
    print(f"\nTuning {scenario['name']} scenario...")
    results = tuner.run_grid_search(scenario['config'], n_samples=n_samples)
    
    # Save best parameters
    best_score, best_params, best_metrics = results[0]
    print(f"\nBest parameters for {scenario['name']}:")
    print("\nProcess Noise Parameters:")
    for k in ['process_noise_pos', 'process_noise_vel', 'process_noise_acc', 
              'process_noise_quat', 'process_noise_omega']:
      print(f"{k}: {best_params[k]:.6f}")
    
    print("\nInitial Uncertainty Parameters:")
    for k in ['pos_uncertainty', 'vel_uncertainty', 'acc_uncertainty', 
              'quat_uncertainty', 'omega_uncertainty']:
      print(f"{k}: {best_params[k]:.6f}")
    
    print(f"\nFinal metrics:")
    print(f"Score: {best_score:.3f}")
    print(f"Position RMSE: {best_metrics['rmse_pos']}")
    print(f"Velocity RMSE: {best_metrics['rmse_vel']}")
    print(f"Orientation RMSE: {best_metrics['rmse_orientation']:.2f}°")
    print(f"Position NES: {best_metrics['pos_nes']:.3f}")
    print(f"Velocity NES: {best_metrics['vel_nes']:.3f}")

    # Save parameters for future use
    params_file = f"{scenario['name'].lower().replace(' ', '_')}_params.txt"
    with open(params_file, 'w') as f:
      for k, v in best_params.items():
          f.write(f"{k}: {v:.6f}\n")

  # Example of parameters to try for each scenario
  straight_params = {
      'process_noise_pos': 0.001,
      'process_noise_vel': 0.01,
      'process_noise_acc': 0.1,
      'process_noise_quat': 0.001,
      'process_noise_omega': 0.01,
      'pos_uncertainty': 10.0,
      'vel_uncertainty': 1.0,
      'acc_uncertainty': 0.1,
      'quat_uncertainty': 0.1,
      'omega_uncertainty': 0.1
  }

  # Add initial parameters to scenarios
  straight_scenario['config']['tuning_params'] = straight_params
