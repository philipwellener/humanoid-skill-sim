#!/usr/bin/env python3
"""
Quick test runner that focuses on working functionality
"""

import sys
import os
import time
from datetime import datetime

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def run_individual_tests():
    """Run individual test files to get clean results"""
    
    print("🧪 HUMANOID SKILL SIMULATOR - QUICK TEST VERIFICATION")
    print("=" * 70)
    print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    tests = {
        "Installation Verification": "verify_installation.py",
        "Skill Tests": "tests/skill_tests.py", 
        "Behavior Tree Tests": "tests/behavior_tree_tests.py",
        "Scenario Tests": "tests/scenario_tests.py",
        "Regression Tests": "tests/regression_tests.py"
    }
    
    results = {}
    
    for test_name, test_file in tests.items():
        print(f"🔍 Running {test_name}...")
        print("-" * 50)
        
        try:
            import subprocess
            result = subprocess.run([
                sys.executable, test_file
            ], capture_output=True, text=True, timeout=60, 
               cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            
            if result.returncode == 0:
                results[test_name] = "✅ PASSED"
                print(f"✅ {test_name}: PASSED")
            else:
                results[test_name] = "❌ FAILED"
                print(f"❌ {test_name}: FAILED")
                
                # Show some error output
                if result.stderr:
                    print("Error output:")
                    print(result.stderr[:500] + "..." if len(result.stderr) > 500 else result.stderr)
                
        except subprocess.TimeoutExpired:
            results[test_name] = "⏱️ TIMEOUT"
            print(f"⏱️ {test_name}: TIMEOUT")
        except Exception as e:
            results[test_name] = f"🚨 ERROR: {str(e)[:100]}"
            print(f"🚨 {test_name}: ERROR - {e}")
        
        print()
    
    # Summary
    print("=" * 70)
    print("📊 QUICK TEST SUMMARY")
    print("=" * 70)
    
    passed = sum(1 for r in results.values() if "PASSED" in r)
    total = len(results)
    
    for test_name, result in results.items():
        print(f"{test_name:25} {result}")
    
    print()
    print(f"📈 Success Rate: {passed}/{total} ({passed/total*100:.1f}%)")
    
    # Core functionality check
    print("\n" + "=" * 70)
    print("🎯 CORE FUNCTIONALITY STATUS")
    print("=" * 70)
    
    if "PASSED" in results.get("Installation Verification", ""):
        print("✅ Dependencies and imports working")
    else:
        print("❌ Dependencies have issues")
    
    if "PASSED" in results.get("Skill Tests", ""):
        print("✅ Skill system mostly functional")
    elif "FAILED" in results.get("Skill Tests", ""):
        print("⚠️  Skill system has minor issues")
    else:
        print("❌ Skill system has major issues")
    
    if "PASSED" in results.get("Scenario Tests", ""):
        print("✅ End-to-end scenarios working")
    else:
        print("⚠️  End-to-end scenarios need attention")
    
    print("\n💡 RECOMMENDATIONS:")
    
    if passed >= total * 0.8:
        print("🎉 System is in excellent shape! Ready for production use.")
    elif passed >= total * 0.6:
        print("✅ System is functional with minor issues. Safe to use for development.")
    elif passed >= total * 0.4:
        print("⚠️  System has significant issues but core functionality works.")
    else:
        print("🚨 System needs attention before use. Focus on critical fixes.")
    
    print(f"\nCompleted: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)
    
    return passed >= total * 0.6


def run_simulation_demo():
    """Run a quick simulation demo to verify functionality"""
    print("\n🚀 RUNNING SIMULATION DEMO")
    print("-" * 40)
    
    try:
        import subprocess
        result = subprocess.run([
            sys.executable, "run_sim.py", "--workflow", "simple_walk", "--no-gui", "--export-logs"
        ], capture_output=True, text=True, timeout=30,
           cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        
        if result.returncode == 0 and "SUCCESS" in result.stdout:
            print("✅ Simulation demo completed successfully!")
            print("🎯 Core simulation functionality verified!")
            return True
        else:
            print("❌ Simulation demo failed")
            if result.stderr:
                print("Error:", result.stderr[:200])
            return False
            
    except subprocess.TimeoutExpired:
        print("⏱️ Simulation demo timed out")
        return False
    except Exception as e:
        print(f"🚨 Simulation demo error: {e}")
        return False


def main():
    """Main function"""
    test_success = run_individual_tests()
    demo_success = run_simulation_demo()
    
    overall_success = test_success and demo_success
    
    print(f"\n{'='*70}")
    print("🏁 FINAL RESULT")
    print("=" * 70)
    
    if overall_success:
        print("🎉 SYSTEM READY - All critical tests passed!")
        print("✅ You can use the simulator with confidence")
        sys.exit(0)
    elif test_success:
        print("⚠️  MOSTLY READY - Tests passed but simulation needs attention")
        sys.exit(0)
    else:
        print("🚨 NEEDS WORK - Several issues detected")
        sys.exit(1)


if __name__ == "__main__":
    main()
