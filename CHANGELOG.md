<!--
SPDX-FileCopyrightText: 2025 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.6.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.6.1...v0.6.2) (2026-06-25)




### Bug Fixes:

* disarm reports failure instead of false success on dead controller (#57) (#63) by James Harton

### Improvements:

* support bb 0.20.3 robot_opts/0 child spec (#52) by James Harton

## [v0.6.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.6.0...v0.6.1) (2026-05-28)




### Bug Fixes:

* bump bb to `~> 0.20`, use `set_robot_param_default` (#39) by James Harton

## [v0.6.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.5.2...v0.6.0) (2026-05-21)




### Features:

* remove `reverse?`, move to motor-space (#35) by James Harton

## [v0.5.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.5.1...v0.5.2) (2026-05-17)




## [v0.5.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.5.0...v0.5.1) (2026-05-13)




### Improvements:

* add `bb_servo_pca9685.install` igniter task (#32) by James Harton

## [v0.5.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.4.0...v0.5.0) (2026-01-11)




### Features:

* migrate to structured error system (#8) by James Harton

### Improvements:

* use structured error for missing OE pin (#9) by James Harton

## [v0.4.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.1...v0.4.0) (2025-12-24)
### Breaking Changes:

* update to bb 0.8 wrapper GenServer pattern (#7) by James Harton



## [v0.3.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.0...v0.3.1) (2025-12-20)


### Improvements

* update for compatibility with BB 0.6.


## [v0.3.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.0...v0.3.0) (2025-12-14)




### Features:

* implement BB.Safety behaviour for safe disarm (#3) by James Harton

## [v0.2.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.1.0...v0.2.0) (2025-12-13)




### Features:

* add PCA9685 servo controller, actuator, and sensor modules by James Harton

### Improvements:

* use standard actuator command interface by James Harton
