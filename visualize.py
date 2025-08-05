"""
Visualization tools for simulation data and behavior tree execution
"""

import matplotlib.pyplot as plt
import numpy as np
import json
from typing import List, Dict, Any
import os


class SimulationVisualizer:
    """Visualization tools for simulation analysis"""
    
    def __init__(self):
        self.fig = None
        self.axes = None
        
    def plot_execution_timeline(self, log_file: str, save_path: str = None) -> None:
        """Plot execution timeline from log file"""
        
        # Load log data
        with open(log_file, 'r') as f:
            log_data = json.load(f)
            
        steps = log_data.get('steps', [])
        if not steps:
            print("No steps found in log file")
            return
            
        # Extract data for plotting
        step_names = []
        step_durations = []
        step_statuses = []
        step_times = []
        
        base_time = None
        for step in steps:
            step_names.append(step.get('skill', 'Unknown'))
            step_durations.append(step.get('duration', 0.0))
            step_statuses.append(step.get('status', 'UNKNOWN'))
            
            # Calculate relative time
            if 'timestamp' in step:
                from datetime import datetime
                timestamp = datetime.fromisoformat(step['timestamp'])
                if base_time is None:
                    base_time = timestamp
                    step_times.append(0.0)
                else:
                    step_times.append((timestamp - base_time).total_seconds())
            else:
                step_times.append(len(step_times))
                
        # Create timeline plot
        self.fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Plot 1: Step durations
        colors = ['green' if status == 'SUCCESS' else 'red' if status == 'FAILURE' else 'orange' 
                 for status in step_statuses]
        
        bars = ax1.bar(range(len(step_names)), step_durations, color=colors, alpha=0.7)
        ax1.set_xlabel('Execution Step')
        ax1.set_ylabel('Duration (seconds)')
        ax1.set_title('Step Execution Durations')
        ax1.set_xticks(range(len(step_names)))
        ax1.set_xticklabels(step_names, rotation=45, ha='right')
        
        # Add status legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='green', alpha=0.7, label='Success'),
            Patch(facecolor='red', alpha=0.7, label='Failure'),
            Patch(facecolor='orange', alpha=0.7, label='Other')
        ]
        ax1.legend(handles=legend_elements)
        
        # Plot 2: Cumulative timeline
        cumulative_times = np.cumsum([0] + step_durations[:-1])
        
        for i, (start_time, duration, status) in enumerate(zip(cumulative_times, step_durations, step_statuses)):
            color = 'green' if status == 'SUCCESS' else 'red' if status == 'FAILURE' else 'orange'
            ax2.barh(i, duration, left=start_time, color=color, alpha=0.7)
            
        ax2.set_yticks(range(len(step_names)))
        ax2.set_yticklabels(step_names)
        ax2.set_xlabel('Cumulative Time (seconds)')
        ax2.set_title('Execution Timeline')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Timeline plot saved to {save_path}")
        else:
            plt.show()
            
    def plot_success_rates(self, log_files: List[str], save_path: str = None) -> None:
        """Plot success rates across multiple runs"""
        
        run_data = []
        
        for log_file in log_files:
            try:
                with open(log_file, 'r') as f:
                    log_data = json.load(f)
                    
                steps = log_data.get('steps', [])
                total_steps = len(steps)
                successful_steps = len([s for s in steps if s.get('status') == 'SUCCESS'])
                
                run_data.append({
                    'run_id': log_data.get('run_id', os.path.basename(log_file)),
                    'success_rate': successful_steps / total_steps if total_steps > 0 else 0.0,
                    'total_steps': total_steps,
                    'failures': log_data.get('failures', 0),
                    'duration': log_data.get('total_duration', 0.0)
                })
            except Exception as e:
                print(f"Error processing {log_file}: {e}")
                
        if not run_data:
            print("No valid log files found")
            return
            
        # Create plots
        self.fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Success rates
        run_ids = [data['run_id'] for data in run_data]
        success_rates = [data['success_rate'] for data in run_data]
        
        bars = ax1.bar(range(len(run_ids)), success_rates, color='skyblue', alpha=0.8)
        ax1.set_xlabel('Run ID')
        ax1.set_ylabel('Success Rate')
        ax1.set_title('Success Rates Across Runs')
        ax1.set_xticks(range(len(run_ids)))
        ax1.set_xticklabels(run_ids, rotation=45, ha='right')
        ax1.set_ylim(0, 1)
        ax1.grid(True, alpha=0.3)
        
        # Add percentage labels on bars
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + 0.01,
                    f'{rate:.1%}', ha='center', va='bottom')
        
        # Plot 2: Total steps vs failures
        total_steps = [data['total_steps'] for data in run_data]
        failures = [data['failures'] for data in run_data]
        
        ax2.scatter(total_steps, failures, color='red', alpha=0.6, s=50)
        ax2.set_xlabel('Total Steps')
        ax2.set_ylabel('Number of Failures')
        ax2.set_title('Failures vs Total Steps')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Execution durations
        durations = [data['duration'] for data in run_data]
        
        ax3.bar(range(len(run_ids)), durations, color='lightgreen', alpha=0.8)
        ax3.set_xlabel('Run ID')
        ax3.set_ylabel('Duration (seconds)')
        ax3.set_title('Execution Durations')
        ax3.set_xticks(range(len(run_ids)))
        ax3.set_xticklabels(run_ids, rotation=45, ha='right')
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Success rate distribution
        ax4.hist(success_rates, bins=10, color='purple', alpha=0.7, edgecolor='black')
        ax4.set_xlabel('Success Rate')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Success Rate Distribution')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Success rate analysis saved to {save_path}")
        else:
            plt.show()
            
    def plot_skill_performance(self, log_files: List[str], save_path: str = None) -> None:
        """Analyze performance of individual skills"""
        
        skill_data = {}
        
        for log_file in log_files:
            try:
                with open(log_file, 'r') as f:
                    log_data = json.load(f)
                    
                for step in log_data.get('steps', []):
                    skill_name = step.get('skill', 'Unknown')
                    status = step.get('status', 'UNKNOWN')
                    duration = step.get('duration', 0.0)
                    
                    if skill_name not in skill_data:
                        skill_data[skill_name] = {
                            'executions': 0,
                            'successes': 0,
                            'failures': 0,
                            'total_duration': 0.0,
                            'durations': []
                        }
                        
                    skill_data[skill_name]['executions'] += 1
                    skill_data[skill_name]['total_duration'] += duration
                    skill_data[skill_name]['durations'].append(duration)
                    
                    if status == 'SUCCESS':
                        skill_data[skill_name]['successes'] += 1
                    elif status == 'FAILURE':
                        skill_data[skill_name]['failures'] += 1
                        
            except Exception as e:
                print(f"Error processing {log_file}: {e}")
                
        if not skill_data:
            print("No skill data found")
            return
            
        # Calculate metrics
        skills = list(skill_data.keys())
        success_rates = []
        avg_durations = []
        execution_counts = []
        
        for skill in skills:
            data = skill_data[skill]
            success_rate = data['successes'] / data['executions'] if data['executions'] > 0 else 0.0
            avg_duration = data['total_duration'] / data['executions'] if data['executions'] > 0 else 0.0
            
            success_rates.append(success_rate)
            avg_durations.append(avg_duration)
            execution_counts.append(data['executions'])
            
        # Create plots
        self.fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Success rates by skill
        bars = ax1.bar(range(len(skills)), success_rates, color='lightblue', alpha=0.8)
        ax1.set_xlabel('Skill')
        ax1.set_ylabel('Success Rate')
        ax1.set_title('Success Rates by Skill Type')
        ax1.set_xticks(range(len(skills)))
        ax1.set_xticklabels(skills, rotation=45, ha='right')
        ax1.set_ylim(0, 1)
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Average durations by skill
        ax2.bar(range(len(skills)), avg_durations, color='orange', alpha=0.8)
        ax2.set_xlabel('Skill')
        ax2.set_ylabel('Average Duration (seconds)')
        ax2.set_title('Average Execution Duration by Skill')
        ax2.set_xticks(range(len(skills)))
        ax2.set_xticklabels(skills, rotation=45, ha='right')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Execution frequency
        ax3.bar(range(len(skills)), execution_counts, color='green', alpha=0.8)
        ax3.set_xlabel('Skill')
        ax3.set_ylabel('Number of Executions')
        ax3.set_title('Skill Execution Frequency')
        ax3.set_xticks(range(len(skills)))
        ax3.set_xticklabels(skills, rotation=45, ha='right')
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Duration distribution for most executed skill
        if skills:
            most_executed_skill = max(skills, key=lambda s: skill_data[s]['executions'])
            durations = skill_data[most_executed_skill]['durations']
            
            ax4.hist(durations, bins=15, color='purple', alpha=0.7, edgecolor='black')
            ax4.set_xlabel('Duration (seconds)')
            ax4.set_ylabel('Frequency')
            ax4.set_title(f'Duration Distribution: {most_executed_skill}')
            ax4.grid(True, alpha=0.3)
            
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Skill performance analysis saved to {save_path}")
        else:
            plt.show()
            
    def plot_robot_trajectory(self, log_file: str, save_path: str = None) -> None:
        """Plot robot trajectory during execution (if position data available)"""
        
        # This would require position logging to be added to the simulation
        # For now, create a placeholder implementation
        
        print("Robot trajectory plotting not yet implemented")
        print("Would require position data logging in simulation steps")
        
        # Placeholder plot
        self.fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Generate sample trajectory data
        t = np.linspace(0, 2*np.pi, 100)
        x = 2 * np.cos(t) + np.random.normal(0, 0.1, 100)
        y = 2 * np.sin(t) + np.random.normal(0, 0.1, 100)
        
        ax.plot(x, y, 'b-', alpha=0.7, linewidth=2, label='Robot Path')
        ax.scatter(x[0], y[0], color='green', s=100, label='Start', zorder=5)
        ax.scatter(x[-1], y[-1], color='red', s=100, label='End', zorder=5)
        
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_title('Robot Trajectory (Sample)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        else:
            plt.show()


def main():
    """Example usage of visualization tools"""
    
    visualizer = SimulationVisualizer()
    
    # Check if log files exist
    log_dir = "logs"
    if os.path.exists(log_dir):
        log_files = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith('.json')]
        
        if log_files:
            print(f"Found {len(log_files)} log files")
            
            # Plot first log file timeline
            visualizer.plot_execution_timeline(log_files[0])
            
            # Plot success rates if multiple files
            if len(log_files) > 1:
                visualizer.plot_success_rates(log_files)
                visualizer.plot_skill_performance(log_files)
        else:
            print("No log files found in logs/ directory")
    else:
        print("Logs directory not found")
        print("Run some simulations first to generate log data")


if __name__ == "__main__":
    main()
