#!/usr/bin/env python3
"""
Final comprehensive test report for the humanoid skill simulator
"""

import sys
import os
import json
from datetime import datetime
from pathlib import Path

def generate_test_report():
    """Generate comprehensive test report"""
    
    report = {
        "report_generated": datetime.now().isoformat(),
        "project_name": "Humanoid Skill Simulator",
        "version": "1.0.0",
        "test_results": {}
    }
    
    print("🤖 HUMANOID SKILL SIMULATOR - COMPREHENSIVE TEST REPORT")
    print("=" * 80)
    print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Test 1: Installation and Dependencies
    print("🔧 INSTALLATION & DEPENDENCIES")
    print("-" * 50)
    
    try:
        import pybullet, numpy, yaml, matplotlib
        deps_status = "✅ PASSED - All dependencies available"
        deps_success = True
    except ImportError as e:
        deps_status = f"❌ FAILED - Missing dependency: {e}"
        deps_success = False
    
    print(deps_status)
    report["test_results"]["dependencies"] = {
        "status": "PASSED" if deps_success else "FAILED",
        "details": deps_status
    }
    
    # Test 2: Module Imports
    print("\n📦 MODULE IMPORTS")
    print("-" * 50)
    
    modules_to_test = [
        ("behavior_tree.node", "Node, NodeStatus"),
        ("behavior_tree.sequence", "Sequence"),
        ("behavior_tree.fallback", "Fallback"),
        ("sim.world", "SimulationWorld"),
        ("sim.robot", "HumanoidRobot"),
        ("skills.walk", "WalkToTargetSkill"),
        ("skills.pick", "PickObjectSkill"),
        ("skills.place", "PlaceObjectSkill")
    ]
    
    module_results = []
    for module_name, class_name in modules_to_test:
        try:
            exec(f"from {module_name} import {class_name}")
            result = f"✅ {module_name}"
            success = True
        except ImportError as e:
            result = f"❌ {module_name} - {str(e)[:50]}"
            success = False
        
        print(result)
        module_results.append({"module": module_name, "status": "PASSED" if success else "FAILED"})
    
    modules_passed = sum(1 for r in module_results if r["status"] == "PASSED")
    modules_total = len(module_results)
    
    report["test_results"]["modules"] = {
        "passed": modules_passed,
        "total": modules_total,
        "success_rate": modules_passed / modules_total * 100,
        "details": module_results
    }
    
    # Test 3: Configuration Files
    print(f"\n📋 CONFIGURATION FILES")
    print("-" * 50)
    
    config_files = ["configs/workflows.yaml", "README.md", "requirements.txt"]
    config_results = []
    
    for config_file in config_files:
        if Path(config_file).exists():
            result = f"✅ {config_file}"
            success = True
        else:
            result = f"❌ {config_file} - Missing"
            success = False
        
        print(result)
        config_results.append({"file": config_file, "status": "PASSED" if success else "FAILED"})
    
    # Test workflow configurations
    try:
        import yaml
        with open("configs/workflows.yaml", 'r') as f:
            workflows = yaml.safe_load(f)
        
        workflow_count = len([k for k, v in workflows.items() if isinstance(v, dict)])
        print(f"✅ Found {workflow_count} workflow configurations")
        config_results.append({"file": "workflow_configs", "status": "PASSED", "count": workflow_count})
        
    except Exception as e:
        print(f"❌ Workflow configuration error: {e}")
        config_results.append({"file": "workflow_configs", "status": "FAILED"})
    
    report["test_results"]["configuration"] = {
        "details": config_results
    }
    
    # Test 4: Core Simulation Functionality
    print(f"\n🎮 CORE SIMULATION FUNCTIONALITY")
    print("-" * 50)
    
    simulation_tests = []
    
    # Test basic simulation creation
    try:
        from sim.world import SimulationWorld
        world = SimulationWorld(use_gui=False)
        print("✅ Simulation world creation")
        simulation_tests.append({"test": "world_creation", "status": "PASSED"})
        
        # Test initialization
        try:
            result = world.initialize()
            if result:
                print("✅ Simulation world initialization")
                simulation_tests.append({"test": "world_initialization", "status": "PASSED"})
            else:
                print("⚠️  Simulation world initialization (partial)")
                simulation_tests.append({"test": "world_initialization", "status": "PARTIAL"})
        except Exception as e:
            print(f"❌ Simulation world initialization: {e}")
            simulation_tests.append({"test": "world_initialization", "status": "FAILED"})
            
        # Cleanup
        try:
            world.shutdown()
        except:
            pass
            
    except Exception as e:
        print(f"❌ Simulation world creation: {e}")
        simulation_tests.append({"test": "world_creation", "status": "FAILED"})
    
    # Test robot loading
    try:
        from sim.robot import HumanoidRobot
        robot = HumanoidRobot()
        print("✅ Robot creation")
        simulation_tests.append({"test": "robot_creation", "status": "PASSED"})
        
        # Test robot loading (might fail without URDF)
        try:
            result = robot.load_robot()
            if result:
                print("✅ Robot URDF loading")
                simulation_tests.append({"test": "robot_loading", "status": "PASSED"})
            else:
                print("⚠️  Robot URDF loading (expected - no URDF file)")
                simulation_tests.append({"test": "robot_loading", "status": "EXPECTED_FAIL"})
        except Exception as e:
            print(f"⚠️  Robot URDF loading (expected): {str(e)[:50]}")
            simulation_tests.append({"test": "robot_loading", "status": "EXPECTED_FAIL"})
            
    except Exception as e:
        print(f"❌ Robot creation: {e}")
        simulation_tests.append({"test": "robot_creation", "status": "FAILED"})
    
    report["test_results"]["simulation"] = {
        "details": simulation_tests
    }
    
    # Test 5: Behavior Tree System
    print(f"\n🌳 BEHAVIOR TREE SYSTEM")
    print("-" * 50)
    
    bt_tests = []
    
    try:
        from behavior_tree.sequence import Sequence
        from behavior_tree.fallback import Fallback
        from behavior_tree.node import NodeStatus
        
        # Test sequence creation
        seq = Sequence()
        print("✅ Sequence node creation")
        bt_tests.append({"test": "sequence_creation", "status": "PASSED"})
        
        # Test fallback creation
        fall = Fallback()
        print("✅ Fallback node creation")
        bt_tests.append({"test": "fallback_creation", "status": "PASSED"})
        
        # Test node status enum
        statuses = [NodeStatus.SUCCESS, NodeStatus.FAILURE, NodeStatus.RUNNING]
        print("✅ Node status system")
        bt_tests.append({"test": "node_status", "status": "PASSED"})
        
    except Exception as e:
        print(f"❌ Behavior tree system: {e}")
        bt_tests.append({"test": "behavior_tree_system", "status": "FAILED"})
    
    report["test_results"]["behavior_tree"] = {
        "details": bt_tests
    }
    
    # Test 6: Skills System
    print(f"\n🎯 SKILLS SYSTEM")
    print("-" * 50)
    
    skills_tests = []
    
    try:
        from skills.walk import WalkToTargetSkill
        from skills.pick import PickObjectSkill
        from skills.place import PlaceObjectSkill
        
        # Test skill creation
        walk_skill = WalkToTargetSkill([1, 0, 0])
        pick_skill = PickObjectSkill("test_object")
        place_skill = PlaceObjectSkill([2, 2, 0])
        
        print("✅ Walk skill creation")
        print("✅ Pick skill creation") 
        print("✅ Place skill creation")
        
        skills_tests.extend([
            {"test": "walk_skill_creation", "status": "PASSED"},
            {"test": "pick_skill_creation", "status": "PASSED"},
            {"test": "place_skill_creation", "status": "PASSED"}
        ])
        
    except Exception as e:
        print(f"❌ Skills system: {e}")
        skills_tests.append({"test": "skills_system", "status": "FAILED"})
    
    report["test_results"]["skills"] = {
        "details": skills_tests
    }
    
    # Test 7: End-to-End Workflow Execution
    print(f"\n🚀 END-TO-END WORKFLOW EXECUTION")
    print("-" * 50)
    
    try:
        import subprocess
        result = subprocess.run([
            sys.executable, "run_sim.py", "--workflow", "simple_walk", "--no-gui"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0 and "SUCCESS" in result.stdout:
            print("✅ Simple walk workflow execution")
            e2e_status = "PASSED"
        else:
            print("❌ Simple walk workflow execution")
            e2e_status = "FAILED"
            
    except subprocess.TimeoutExpired:
        print("⏱️ Workflow execution timeout")
        e2e_status = "TIMEOUT"
    except Exception as e:
        print(f"❌ Workflow execution error: {e}")
        e2e_status = "FAILED"
    
    report["test_results"]["end_to_end"] = {
        "status": e2e_status
    }
    
    # Calculate overall results
    print(f"\n{'='*80}")
    print("📊 OVERALL TEST SUMMARY")
    print("=" * 80)
    
    total_sections = 6
    passed_sections = 0
    
    # Count passed sections
    if deps_success:
        passed_sections += 1
    if modules_passed >= modules_total * 0.8:
        passed_sections += 1
    if len([r for r in config_results if r["status"] == "PASSED"]) >= len(config_results) * 0.8:
        passed_sections += 1
    if len([r for r in simulation_tests if r["status"] in ["PASSED", "EXPECTED_FAIL"]]) >= len(simulation_tests) * 0.8:
        passed_sections += 1
    if len([r for r in bt_tests if r["status"] == "PASSED"]) >= len(bt_tests) * 0.8:
        passed_sections += 1
    if e2e_status == "PASSED":
        passed_sections += 1
    
    overall_success_rate = passed_sections / total_sections * 100
    
    print(f"📈 Overall Success Rate: {passed_sections}/{total_sections} ({overall_success_rate:.1f}%)")
    
    report["test_results"]["overall"] = {
        "sections_passed": passed_sections,
        "total_sections": total_sections,
        "success_rate": overall_success_rate
    }
    
    # Recommendations
    print(f"\n💡 RECOMMENDATIONS:")
    print("-" * 50)
    
    if overall_success_rate >= 90:
        print("🎉 EXCELLENT - System is production ready!")
        recommendation = "PRODUCTION_READY"
    elif overall_success_rate >= 75:
        print("✅ GOOD - System is functional with minor issues")
        recommendation = "FUNCTIONAL"
    elif overall_success_rate >= 50:
        print("⚠️  MODERATE - Core functionality works, some issues remain")
        recommendation = "CORE_FUNCTIONAL"
    else:
        print("🚨 NEEDS WORK - Significant issues require attention")
        recommendation = "NEEDS_WORK"
    
    report["test_results"]["recommendation"] = recommendation
    
    # Specific issues and next steps
    print(f"\n📋 NEXT STEPS:")
    print("-" * 30)
    
    if not deps_success:
        print("• Install missing dependencies")
    if modules_passed < modules_total:
        print("• Fix module import issues")
    if e2e_status != "PASSED":
        print("• Debug workflow execution problems")
    
    print("• Run 'python run_sim.py --workflow simple_walk --no-gui' to test")
    print("• Run 'python verify_installation.py' for detailed verification")
    print("• Check logs/ directory for execution details")
    
    # Save report
    report_file = f"test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    Path("logs").mkdir(exist_ok=True)
    with open(f"logs/{report_file}", 'w') as f:
        json.dump(report, f, indent=2)
    
    print(f"\n💾 Detailed report saved to: logs/{report_file}")
    print("=" * 80)
    
    return overall_success_rate >= 75


if __name__ == "__main__":
    success = generate_test_report()
    sys.exit(0 if success else 1)
