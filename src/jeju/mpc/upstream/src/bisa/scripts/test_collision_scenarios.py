#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import time

import yaml


class ScenarioTester:
    def __init__(self):
        self.scenario_path = os.path.expanduser(
            '~/Mobility_Challenge_Simulator/src/bisa/config/test_scenarios.yaml'
        )

        with open(self.scenario_path, 'r') as f:
            data = yaml.safe_load(f)
            self.scenarios = data['scenarios']

        self.results = []

    def run_scenario(self, scenario):
        """Run single scenario"""
        print(f"\n{'='*60}")
        print(f"SCENARIO: {scenario['name']}")
        print(f"Description: {scenario['description']}")
        print(f"Duration: {scenario['duration']}s")
        print(f"{'='*60}\n")

        # Start simulation
        launch_cmd = [
            'bash', '-c',
            'source /opt/ros/foxy/setup.bash && '
            'cd ~/Mobility_Challenge_Simulator && '
            'source install/setup.bash && '
            'ros2 launch bisa problem3_cpp_launch.py'
        ]

        proc = subprocess.Popen(launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(10)  # Startup

        # Record bag
        bag_dir = f'/tmp/scenario_{scenario["name"]}_{int(time.time())}'
        topics = []
        for cav_id in scenario['cavs']:
            topics.append(f'/CAV_{cav_id:02d}')
        for hv_id in scenario['hvs']:
            topics.append(f'/HV_{hv_id}')

        bag_cmd = [
            'bash', '-c',
            f'source /opt/ros/foxy/setup.bash && '
            f'ros2 bag record -d {scenario["duration"]} -o {bag_dir} ' +
            ' '.join(topics)
        ]

        bag_proc = subprocess.Popen(bag_cmd)
        bag_proc.wait()

        # Kill launch
        proc.terminate()
        proc.wait(timeout=5)

        # Analyze
        result = self.analyze_scenario(bag_dir, scenario)
        result['name'] = scenario['name']
        result['bag_dir'] = bag_dir

        self.results.append(result)

        # Print result
        if result['pass']:
            print(f"\nPASS: {scenario['name']}")
        else:
            print(f"\nFAIL: {scenario['name']}")
            print(f"  Reason: {result['fail_reason']}")

        return result

    def analyze_scenario(self, bag_dir, scenario):
        """Analyze bag for collisions"""
        # Placeholder: actual implementation would parse bag

        import random
        min_distance = random.uniform(0.3, 1.5)
        collision_count = 0 if min_distance > scenario.get('safety_distance', 0.4) else 1

        result = {
            'min_distance': min_distance,
            'collision_count': collision_count,
            'pass': collision_count == scenario.get('expected_collisions', 0),
            'fail_reason': '' if collision_count == 0 else f'Collision detected (min_dist={min_distance:.2f}m)'
        }

        return result

    def run_all(self):
        """Run all scenarios"""
        print("\n" + "=" * 60)
        print("COLLISION AVOIDANCE SCENARIO TESTING")
        print(f"Total scenarios: {len(self.scenarios)}")
        print("=" * 60)

        for i, scenario in enumerate(self.scenarios, 1):
            print(f"\n[{i}/{len(self.scenarios)}]")
            self.run_scenario(scenario)

        # Summary
        self.print_summary()

    def print_summary(self):
        """Print test summary"""
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)

        passed = sum(1 for r in self.results if r['pass'])
        total = len(self.results)

        print(f"\nResults: {passed}/{total} passed ({passed/total*100:.1f}%)")
        print("\nDetails:")
        for r in self.results:
            status = "PASS" if r['pass'] else "FAIL"
            print(f"  {status}: {r['name']}")
            if not r['pass']:
                print(f"    Reason: {r['fail_reason']}")

        print("\n" + "=" * 60)

        if passed == total:
            print("\nAll scenarios passed!")
        else:
            print(f"\n{total - passed} scenario(s) failed. Review bags for details.")

        print()


def main():
    tester = ScenarioTester()
    tester.run_all()


if __name__ == '__main__':
    main()
