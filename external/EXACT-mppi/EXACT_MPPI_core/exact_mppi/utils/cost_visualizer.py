"""
Cost Visualization and Analysis Tool for MPPI Controller

This module provides real-time visualization and analysis of individual cost components
to facilitate parameter tuning.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from collections import deque
from typing import Dict, Optional, List
import time


class CostVisualizer:
    """
    Real-time visualization of MPPI cost components for parameter tuning.
    
    Features:
    - Real-time cost component plots
    - Cost breakdown pie chart
    - Time-series history of each cost
    - Statistical analysis (mean, std, min, max)
    - Correlation analysis between costs and performance
    """
    
    def __init__(self, max_history: int = 200, update_interval: int = 5):
        """
        Initialize the cost visualizer.
        
        Args:
            max_history: Maximum number of timesteps to keep in history
            update_interval: Update plot every N steps (to reduce overhead)
        """
        self.max_history = max_history
        self.update_interval = update_interval
        self.step_counter = 0
        
        # Cost history storage
        self.cost_history = {
            'control': deque(maxlen=max_history),
            'speed_toward': deque(maxlen=max_history),
            'ref_speed': deque(maxlen=max_history),
            'collision': deque(maxlen=max_history),
            'ref_traj': deque(maxlen=max_history),
            'goal_pos': deque(maxlen=max_history),
            'goal_angle': deque(maxlen=max_history),
            'total': deque(maxlen=max_history),
            'goal_distance': deque(maxlen=max_history),
            'min_obstacle_dist': deque(maxlen=max_history),
        }
        
        # Statistics
        self.stats = {}
        
        # Matplotlib setup
        self.fig = None
        self.axes = {}
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the matplotlib figure with multiple subplots."""
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('MPPI Cost Analysis Dashboard', fontsize=16, fontweight='bold')
        
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # Time series plots for individual costs
        self.axes['control'] = self.fig.add_subplot(gs[0, 0])
        self.axes['goal'] = self.fig.add_subplot(gs[0, 1])
        self.axes['collision'] = self.fig.add_subplot(gs[0, 2])
        
        self.axes['speed'] = self.fig.add_subplot(gs[1, 0])
        self.axes['ref_traj'] = self.fig.add_subplot(gs[1, 1])
        self.axes['total'] = self.fig.add_subplot(gs[1, 2])
        
        # Cost breakdown pie chart
        self.axes['pie'] = self.fig.add_subplot(gs[2, 0])
        
        # State information (goal distance, obstacle distance)
        self.axes['state'] = self.fig.add_subplot(gs[2, 1])
        
        # Statistics table
        self.axes['stats'] = self.fig.add_subplot(gs[2, 2])
        self.axes['stats'].axis('off')
        
        # Configure axes
        for key, ax in self.axes.items():
            if key not in ['pie', 'stats']:
                ax.grid(True, alpha=0.3)
                ax.set_xlabel('Time Step')
                
    def update(self, cost_breakdown: Dict[str, float], state_info: Optional[Dict] = None):
        """
        Update visualization with new cost data.
        
        Args:
            cost_breakdown: Dictionary with individual cost components
            state_info: Optional dict with goal_distance, min_obstacle_dist, etc.
        """
        self.step_counter += 1
        
        # Store data
        for key in self.cost_history.keys():
            if key in cost_breakdown:
                self.cost_history[key].append(cost_breakdown[key])
            elif state_info and key in state_info:
                self.cost_history[key].append(state_info[key])
            else:
                if len(self.cost_history[key]) > 0:
                    self.cost_history[key].append(self.cost_history[key][-1])
                else:
                    self.cost_history[key].append(0.0)
        
        # Update plot periodically
        if self.step_counter % self.update_interval == 0:
            self._update_plots()
            
    def _update_plots(self):
        """Internal method to update all plots."""
        steps = np.arange(len(self.cost_history['total']))
        
        # Clear all axes
        for ax in self.axes.values():
            ax.clear()
            
        # 1. Control cost
        ax = self.axes['control']
        if len(self.cost_history['control']) > 0:
            ax.plot(steps, list(self.cost_history['control']), 'b-', linewidth=2, label='Control')
            ax.fill_between(steps, 0, list(self.cost_history['control']), alpha=0.3)
            ax.set_title('Control Cost (Smoothness)', fontweight='bold')
            ax.set_ylabel('Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 2. Goal costs
        ax = self.axes['goal']
        if len(self.cost_history['goal_pos']) > 0:
            ax.plot(steps, list(self.cost_history['goal_pos']), 'g-', linewidth=2, label='Position')
            ax.plot(steps, list(self.cost_history['goal_angle']), 'orange', linewidth=2, label='Angle')
            ax.set_title('Goal Arrival Costs (Nav2)', fontweight='bold')
            ax.set_ylabel('Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 3. Collision cost
        ax = self.axes['collision']
        if len(self.cost_history['collision']) > 0:
            ax.plot(steps, list(self.cost_history['collision']), 'r-', linewidth=2, label='Collision')
            ax.fill_between(steps, 0, list(self.cost_history['collision']), alpha=0.3, color='red')
            ax.set_title('Collision Avoidance Cost', fontweight='bold')
            ax.set_ylabel('Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 4. Speed costs
        ax = self.axes['speed']
        if len(self.cost_history['speed_toward']) > 0:
            ax.plot(steps, list(self.cost_history['speed_toward']), 'purple', linewidth=2, label='Speed Toward Goal')
            ax.plot(steps, list(self.cost_history['ref_speed']), 'cyan', linewidth=2, label='Reference Speed')
            ax.set_title('Speed Regulation Costs', fontweight='bold')
            ax.set_ylabel('Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 5. Reference trajectory cost
        ax = self.axes['ref_traj']
        if len(self.cost_history['ref_traj']) > 0:
            ax.plot(steps, list(self.cost_history['ref_traj']), 'm-', linewidth=2, label='Ref Trajectory')
            ax.fill_between(steps, 0, list(self.cost_history['ref_traj']), alpha=0.3, color='magenta')
            ax.set_title('Reference Trajectory Tracking', fontweight='bold')
            ax.set_ylabel('Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 6. Total cost
        ax = self.axes['total']
        if len(self.cost_history['total']) > 0:
            ax.plot(steps, list(self.cost_history['total']), 'k-', linewidth=3, label='Total Cost')
            ax.fill_between(steps, 0, list(self.cost_history['total']), alpha=0.2)
            ax.set_title('Total Cost Evolution', fontweight='bold')
            ax.set_ylabel('Total Cost')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
        # 7. Cost breakdown pie chart (recent average)
        ax = self.axes['pie']
        if len(self.cost_history['total']) > 0:
            recent_window = min(20, len(steps))
            costs = {
                'Control': np.mean(list(self.cost_history['control'])[-recent_window:]),
                'Goal Pos': np.mean(list(self.cost_history['goal_pos'])[-recent_window:]),
                'Goal Angle': np.mean(list(self.cost_history['goal_angle'])[-recent_window:]),
                'Collision': np.mean(list(self.cost_history['collision'])[-recent_window:]),
                'Speed': np.mean(list(self.cost_history['speed_toward'])[-recent_window:]) + 
                         np.mean(list(self.cost_history['ref_speed'])[-recent_window:]),
                'Ref Traj': np.mean(list(self.cost_history['ref_traj'])[-recent_window:]),
            }
            
            # Filter out near-zero costs for cleaner visualization
            costs = {k: abs(v) for k, v in costs.items() if abs(v) > 0.001}
            
            if costs:
                colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#FFA07A', '#98D8C8', '#F7DC6F']
                ax.pie(costs.values(), labels=costs.keys(), autopct='%1.1f%%', 
                       startangle=90, colors=colors[:len(costs)])
                ax.set_title(f'Cost Breakdown (Last {recent_window} steps)', fontweight='bold')
            
        # 8. State information
        ax = self.axes['state']
        if len(self.cost_history['goal_distance']) > 0:
            ax.plot(steps, list(self.cost_history['goal_distance']), 'g-', linewidth=2, 
                   label='Goal Distance', marker='o', markersize=3)
            ax2 = ax.twinx()
            ax2.plot(steps, list(self.cost_history['min_obstacle_dist']), 'r-', linewidth=2,
                    label='Min Obstacle Dist', marker='s', markersize=3)
            ax.set_title('State Information', fontweight='bold')
            ax.set_ylabel('Goal Distance (m)', color='g')
            ax2.set_ylabel('Obstacle Distance (m)', color='r')
            ax.tick_params(axis='y', labelcolor='g')
            ax2.tick_params(axis='y', labelcolor='r')
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper left')
            ax2.legend(loc='upper right')
            
        # 9. Statistics table
        ax = self.axes['stats']
        ax.axis('off')
        
        # Compute statistics
        stats_text = "Cost Statistics (Recent 50 steps)\n" + "="*40 + "\n"
        recent = min(50, len(steps))
        
        for name in ['control', 'goal_pos', 'goal_angle', 'collision', 'total']:
            if len(self.cost_history[name]) >= recent:
                data = list(self.cost_history[name])[-recent:]
                stats_text += f"\n{name.replace('_', ' ').title()}:\n"
                stats_text += f"  Mean: {np.mean(data):.4f}\n"
                stats_text += f"  Std:  {np.std(data):.4f}\n"
                stats_text += f"  Max:  {np.max(data):.4f}\n"
                
        ax.text(0.1, 0.9, stats_text, transform=ax.transAxes, 
               fontsize=9, verticalalignment='top', fontfamily='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
        
    def save_figure(self, filename: str = 'cost_analysis.png'):
        """Save the current figure to file."""
        if self.fig:
            self.fig.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Cost analysis figure saved to {filename}")
            
    def close(self):
        """Close the visualization."""
        if self.fig:
            plt.close(self.fig)
            
    def get_statistics(self) -> Dict:
        """Get statistical summary of costs."""
        stats = {}
        for name, history in self.cost_history.items():
            if len(history) > 0:
                data = np.array(list(history))
                stats[name] = {
                    'mean': float(np.mean(data)),
                    'std': float(np.std(data)),
                    'min': float(np.min(data)),
                    'max': float(np.max(data)),
                    'recent_mean': float(np.mean(data[-20:])) if len(data) >= 20 else float(np.mean(data)),
                }
        return stats
    
    def analyze_tuning_suggestions(self) -> List[str]:
        """
        Analyze cost patterns and suggest parameter tuning.
        
        Returns:
            List of tuning suggestions as strings
        """
        suggestions = []
        stats = self.get_statistics()
        
        if 'collision' in stats:
            if stats['collision']['mean'] > 5.0:
                suggestions.append("⚠️  HIGH COLLISION COST: Increase w_collision or reduce d_safe")
            elif stats['collision']['mean'] < 0.1:
                suggestions.append("✓  Collision cost is low - obstacle avoidance working well")
                
        if 'goal_pos' in stats and 'goal_distance' in stats:
            if stats['goal_distance']['recent_mean'] < 1.4 and stats['goal_pos']['mean'] > 2.0:
                suggestions.append("⚠️  HIGH GOAL POSITION COST: Robot struggling to reach goal. Try increasing w_goal_traj_pos")
            elif stats['goal_distance']['recent_mean'] < 0.5 and stats['goal_pos']['mean'] < 0.5:
                suggestions.append("✓  Good goal convergence with Nav2 position critic")
                
        if 'goal_angle' in stats and 'goal_distance' in stats:
            if stats['goal_distance']['recent_mean'] < 0.5 and stats['goal_angle']['mean'] > 1.0:
                suggestions.append("⚠️  HIGH GOAL ANGLE COST: Difficulty aligning orientation. Try increasing w_goal_traj_angle")
                
        if 'control' in stats:
            if stats['control']['std'] > 1.0:
                suggestions.append("⚠️  HIGH CONTROL VARIANCE: Jerky motion detected. Consider increasing w_control")
            elif stats['control']['mean'] < 0.01:
                suggestions.append("✓  Smooth control - w_control tuned well")
                
        if 'ref_traj' in stats and stats['ref_traj']['mean'] > 0:
            if stats['ref_traj']['mean'] > 5.0:
                suggestions.append("⚠️  HIGH REF TRAJECTORY COST: Robot deviating from path. Increase w_ref_traj or check path validity")
                
        return suggestions if suggestions else ["✓  All costs appear reasonably balanced!"]


class CostLogger:
    """Simple logger to save cost data to file for offline analysis."""
    
    def __init__(self, filename: str = 'cost_log.csv'):
        self.filename = filename
        self.file = None
        self.header_written = False
        
    def __enter__(self):
        self.file = open(self.filename, 'w')
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.file:
            self.file.close()
            
    def log(self, step: int, cost_breakdown: Dict, state_info: Optional[Dict] = None):
        """Log cost data to CSV file."""
        if not self.header_written:
            # Write header
            keys = ['step'] + list(cost_breakdown.keys())
            if state_info:
                keys += list(state_info.keys())
            self.file.write(','.join(keys) + '\n')
            self.header_written = True
            
        # Write data
        values = [str(step)]
        values += [str(cost_breakdown.get(k, 0.0)) for k in cost_breakdown.keys()]
        if state_info:
            values += [str(state_info.get(k, 0.0)) for k in state_info.keys()]
        self.file.write(','.join(values) + '\n')
        self.file.flush()
