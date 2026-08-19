"""
============================================================
obstacle_challenge_navigator.py — WallFollower + CornerTurnController
+ PillarAvoider একসাথে মিলিয়ে Obstacle Challenge navigation
============================================================
প্রতিটা scan-এ সিদ্ধান্তের অগ্রাধিকার:
  ১. corner turn চলমান থাকলে -- সেটা আগে শেষ করা (মাঝপথে থামানো হয়
     না, নাহলে robot করিডোরের মাঝে অর্ধেক ঘোরা অবস্থায় আটকে যাবে)
  ২. pillar avoidance -- সামনে pillar থাকলে সঠিক পাশ দিয়ে পার হওয়া
     (corner-turn trigger চেক করার আগেই, যাতে pillar-কে ভুল করে
     "corner চলে এসেছে" (front wall) ধরে ফেলা না হয়)
  ৩. corner turn trigger -- pillar avoidance সক্রিয় না থাকলে করিডোরের
     শেষে corner এসেছে কিনা চেক করা
  ৪. স্বাভাবিক wall-following
============================================================
"""


class ObstacleChallengeNavigator:
    def __init__(self, wall_follower, corner_turn, pillar_avoider):
        self.wall_follower = wall_follower
        self.corner_turn = corner_turn
        self.pillar_avoider = pillar_avoider

    def reset(self):
        self.wall_follower.reset()
        self.pillar_avoider.reset()

    def compute_steering_angle(self, scan, frame, current_yaw, calibration):
        if self.corner_turn.is_turning():
            angle = self.corner_turn.get_turn_angle()
            if self.corner_turn.check_turn_complete(current_yaw):
                self.wall_follower.reset()
                self.pillar_avoider.reset()
            return angle

        pillar_angle = self.pillar_avoider.compute_steering_angle(
            scan, frame, calibration, self.wall_follower)
        if pillar_angle is not None:
            return pillar_angle

        if self.corner_turn.should_start_turn(scan):
            self.corner_turn.start_turn(scan, current_yaw)
            return self.corner_turn.get_turn_angle()

        return self.wall_follower.compute_steering_angle(scan)
