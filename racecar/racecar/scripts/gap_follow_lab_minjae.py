#!/usr/bin/env python
from __future__ import print_function
import sys
from math import pi, atan, sin, cos
import numpy as np
import itertools, operator
# from Wall_following_lab import VELOCITY

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

np.set_printoptions(threshold=sys.maxsize)


prev_steering_angle = 0
t_prev = 0.0
prev_range = []
VELOCITY = 1.5
gain = 0.75
bubble_radius = 170  # LESS THAN 300
n = 80  # consecutive sequence
threshold_distance = 1.5  # threshold distance

class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        # drive_topic = '/vesc/low_level/ackermann_cmd_mux/output'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

        self.drive_msg = AckermannDriveStamped()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)  # None #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=5)  # TODO
        self.angle_list_size = 10
        self.angle_list = list(np.ones(self.angle_list_size) * 540)
        self.loop = 0

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """

        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        # ------------------------- Gap following --------------------------- #
        # self.loop += 1
        # print()
        # print("loop: ", self.loop)
        distance = data.ranges

        distance_data = np.array(distance)

        NUM_RANGES = len(distance_data)

        unit_angle = 270 / NUM_RANGES * pi / 180

        # Safety bubble
        nearest_idx = distance_data.argmin()
        condition = 0
        if nearest_idx < bubble_radius:
            distance_data[0:nearest_idx + bubble_radius] = 0
            # condition = 1
        elif nearest_idx + bubble_radius > len(distance_data):
            distance_data[nearest_idx - bubble_radius:] = 0
            # condition = 2
        else:
            distance_data[nearest_idx - bubble_radius:nearest_idx + bubble_radius] = 0
            # condition = 3

        # print("bubble_radius", bubble_radius)
        # print("condition", condition)
        # print("nearest_idx: ", nearest_idx)

        infinites = np.where(distance_data > threshold_distance, np.inf, 0)
        distance_data = distance_data + infinites
        print("distance data:", list(distance_data))
        # Take index from above distance_data which has infinity
        infinites_idx = np.where(distance_data == np.inf)
        # print("case1:", infinites_idx)
        # ???
        infinites_idx = infinites_idx[0]
        # print("infinites indexes", infinites_idx)

        # ===============================================================================
        """
        Extract cadidate indexes
        input: infinites_idx 
        output: cadidate_idx
        
        [example]
        infinites_idx:
         0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17
          18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35
         448 449 450 451 452 453 454 455 456 457 458 459 460 472 473 474 475 476
         477 478 479 480 481 482 483 484 485 486 487 488 489 490 491 492 493 494
         495 496 497 498 499 500 501 502 503 504 505 506 507 508 509 510 511 512
         513 514 515 516 517 518 519 520 521 522 523 524 525 526 527 528 529 530
         531 532 533 534 535 536 537 538 539 540 541 542 543 544 545 546 547 548
         549 550 551 552 553 554 555 556 557 558 559 560 561 562 563 564 565 566
         567 568 569 570 571 572 573 574 575 576 577 578 579 580 581 582 583 584
         585 586 587 588 589 590 591 592 593 594 595 596 597 598 599 600 601 602
         603 604 605 606 607 608 609 610 611 612 613 614 615 616 617 618 619 620
         621 622 623 624 625 626 627 628 629 630 631 632 633 634 635 636 637 638
         639 640 641
         
         cadidate_idx:
         [[0, 35], [448, 460], [472, 641]]
        """
        candidate_idx = []
        if len(infinites_idx) > 0:
            candidate_idx.append([infinites_idx[0]])
            idx = infinites_idx[0]
            prev_idx = idx
            print()
            for i in range(len(infinites_idx)):
                if infinites_idx[i] == idx:
                    pass
                else:
                    candidate_idx[-1].append(prev_idx)
                    idx = infinites_idx[i]
                    candidate_idx.append([idx])
                    pass
                prev_idx = idx
                idx += 1
            candidate_idx[-1].append(infinites_idx[-1])
            print(candidate_idx)
            # ===============================================================================

            max_gap = 0
            pick_candidate = np.inf
            for i in range(len(candidate_idx)):
                gap = candidate_idx[i][1] - candidate_idx[i][0]
                if gap > max_gap:
                    max_gap = gap
                    pick_candidate = i

            final_gap = candidate_idx[pick_candidate]
            final_angle_idx = int((final_gap[1] + final_gap[0]) / 2)
            print("final_gap:", final_gap)

        else:
            final_angle_idx = 540

        print("final_angle_idx:", final_angle_idx)
        # self.angle_list.append(final_angle_idx)
        # self.angle_list = self.angle_list[1:]
        steering_angle = (final_angle_idx - 540) / 4 * (np.pi / 180)

        # steering_angle = (final_angle_idx - 540) / 4 * (np.pi / 180)
        self.drive_msg.drive.speed = VELOCITY
        if steering_angle * gain > 0.61:
            self.drive_msg.drive.steering_angle = 0.61
        elif steering_angle * gain < -0.61:
            self.drive_msg.drive.steering_angle = -0.61
        else:
            self.drive_msg.drive.steering_angle = steering_angle * gain
        print("steering_angle * gain", steering_angle * gain)
        print("self.drive_msg.drive.steering_angle", self.drive_msg.drive.steering_angle)
        self.drive_pub.publish(self.drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
