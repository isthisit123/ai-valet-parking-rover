import unittest

from scripts.diagnose_aruco_tracking import summarize


class ArucoDiagnosticsTest(unittest.TestCase):
    def test_arrival_reset_counters_are_not_added_again_on_each_get(self):
        samples = [{'pose': {'arrival_test': {'started_at': start, 'status': 'holding',
                                            'hold_reset_counts': {'position_unstable': count}}}}
                   for start, count in [('A', 2), ('A', 2), ('A', 3), ('B', 1)]]
        result = summarize(samples)
        self.assertEqual(result['arrival_hold_reset_counts'], {'position_unstable': 4})
        self.assertEqual(result['arrival_poll_status_counts'], {'holding': 4})

    def test_repeated_get_and_pose_expiry_do_not_inflate_detection_failure_rate(self):
        detected = {'received_monotonic_s': 10, 'frame_id': '1', 'detection_status': 'detected',
                    'tracking_status': 'detected', 'processing_ms': 20}
        samples = [{'pose': detected}, {'pose': {**detected, 'tracking_status': 'stale'}},
                   {'pose': {'received_monotonic_s': 10.1, 'frame_id': '2', 'detection_status': 'lost',
                             'tracking_status': 'lost', 'processing_ms': 40}}, {'error': 'connection failed'}]
        result = summarize(samples)
        self.assertEqual(result['unique_frames'], 2)
        self.assertEqual(result['detection_ratio'], .5)
        self.assertAlmostEqual(result['server_receive_interval_ms']['median'], 100)
        self.assertEqual(result['poll_tracking_status_counts']['stale'], 1)
        self.assertEqual(result['poll_tracking_status_counts']['request_error'], 1)

    def test_not_started_is_no_camera_evidence_and_session_switch_is_not_frame_gap(self):
        self.assertIsNone(summarize([{'pose': {'tracking_status': 'not_started'}}])['detection_ratio'])
        frames = [{'pose': {'received_monotonic_s': i, 'frame_id': '1', 'camera_source': {'session_id': str(i)},
                            'tracking_status': 'lost', 'detection_status': 'lost'}} for i in (10, 20)]
        self.assertIsNone(summarize(frames)['server_receive_interval_ms'])
