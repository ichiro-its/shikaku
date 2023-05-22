#!/usr/bin/env python3

from shikaku.pybullet_subscriber import runPybulletVisualization

def main(args=None):
    try:
        runPybulletVisualization(args=args)
    except rclpy.exceptions.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()