# Changelog

## [3.0.1](https://github.com/utiasDSL/crisp_py/compare/v3.0.0...v3.0.1) (2025-12-08)


### Bug Fixes

* ensure quaternion sign always the same ([8b82f2d](https://github.com/utiasDSL/crisp_py/commit/8b82f2dbba1a97fed45fe4b4affa903bfb637935))

## [3.0.0](https://github.com/utiasDSL/crisp_py/compare/v2.1.0...v3.0.0) (2025-11-12)


### ⚠ BREAKING CHANGES

* use of prefix for joint is now toggleable AND is set to false as default (Breaking Changes)

### Features

* use of prefix for joint is now toggleable AND is set to false as default (Breaking Changes) ([a572c67](https://github.com/utiasDSL/crisp_py/commit/a572c67e1e38156edf90e26bc3fd69d003c34a78))

## [2.1.0](https://github.com/utiasDSL/crisp_py/compare/v2.0.0...v2.1.0) (2025-11-11)


### Features

* Add an optional reset sensor option as a ROS service ([#56](https://github.com/utiasDSL/crisp_py/issues/56)) ([40a575b](https://github.com/utiasDSL/crisp_py/commit/40a575b0d5a949fe71fc1f2e2d463e93deef89b4))

## [2.0.0](https://github.com/utiasDSL/crisp_py/compare/v1.9.0...v2.0.0) (2025-11-02)


### ⚠ BREAKING CHANGES

* Add orientation representation object + Mock scripts for debugging purposes + Simplify Sensors + some refactoring ([#54](https://github.com/utiasDSL/crisp_py/issues/54))

### Features

* Add orientation representation object + Mock scripts for debugging purposes + Simplify Sensors + some refactoring ([#54](https://github.com/utiasDSL/crisp_py/issues/54)) ([e897cc0](https://github.com/utiasDSL/crisp_py/commit/e897cc064af54be4a4c823ce03bc697e1994b00c))

## [1.9.0](https://github.com/utiasDSL/crisp_py/compare/v1.8.0...v1.9.0) (2025-10-31)


### Features

* Add image cropping and minor improvements to gripper and robot ([#52](https://github.com/utiasDSL/crisp_py/issues/52)) ([9335bcb](https://github.com/utiasDSL/crisp_py/commit/9335bcbec171e12f930c1275063aa747ef038912))

## [1.8.0](https://github.com/utiasDSL/crisp_py/compare/v1.7.0...v1.8.0) (2025-10-09)


### Features

* Add factories for different classes so that they can be loaded from yaml files ([#49](https://github.com/utiasDSL/crisp_py/issues/49)) ([2f8f734](https://github.com/utiasDSL/crisp_py/commit/2f8f734fd8c3db132c336455bfc3af64600ec000))

## [1.7.0](https://github.com/utiasDSL/crisp_py/compare/v1.6.1...v1.7.0) (2025-10-09)


### Features

* Add a better error logging if topics are not available ([#47](https://github.com/utiasDSL/crisp_py/issues/47)) ([231612c](https://github.com/utiasDSL/crisp_py/commit/231612c25e09ecbc69756cdda2ae3078707985cd))

## [1.6.1](https://github.com/utiasDSL/crisp_py/compare/v1.6.0...v1.6.1) (2025-10-06)


### Bug Fixes

* Update gripper example + add new controller configs for tests ([5d6e10b](https://github.com/utiasDSL/crisp_py/commit/5d6e10b6ef6e3d5fcd1864cb100c13061db03ad6))

## [1.6.0](https://github.com/utiasDSL/crisp_py/compare/v1.5.1...v1.6.0) (2025-10-04)


### Features

* added a function to Camera to check if the images changed since the last retrieval ([44094b9](https://github.com/utiasDSL/crisp_py/commit/44094b99ea550a85ed59e66992d6bf182e2a802e))

## [1.5.1](https://github.com/utiasDSL/crisp_py/compare/v1.5.0...v1.5.1) (2025-09-10)


### Bug Fixes

* gripper manual calibration had a wrong arguument - index ([db211e5](https://github.com/utiasDSL/crisp_py/commit/db211e5aca6a59d32bc7c690bf4fe594b2b610f5))

## [1.5.0](https://github.com/utiasDSL/crisp_py/compare/v1.4.0...v1.5.0) (2025-09-08)


### Features

* Add twist monitoring in robot object ([738bd86](https://github.com/utiasDSL/crisp_py/commit/738bd8671bb48270f10a0503da6be88c7e213a44))
* added twist to robot ([651adda](https://github.com/utiasDSL/crisp_py/commit/651addac585ea3cf593866f2d045c3ef37865baa))
* move Pose and Twist classes to utils/geometry.py ([89ec5a5](https://github.com/utiasDSL/crisp_py/commit/89ec5a59fbfe9e41029b238206031606224898d1))

## [1.4.0](https://github.com/utiasDSL/crisp_py/compare/v1.3.1...v1.4.0) (2025-08-19)


### Features

* add sensor factory + mistake in wrench sensor ([3f3e951](https://github.com/utiasDSL/crisp_py/commit/3f3e951575cae5e957cc384c6b28812406574257))

## [1.3.1](https://github.com/utiasDSL/crisp_py/compare/v1.3.0...v1.3.1) (2025-08-13)


### Bug Fixes

* example with IIWA still used pinochio ([8800eb0](https://github.com/utiasDSL/crisp_py/commit/8800eb09ed7097a60b5b7d49dc702bbe8514a230))
* fix examples ([262d779](https://github.com/utiasDSL/crisp_py/commit/262d779c40b0e7a568027ff12439cc56a38c233f))

## [1.3.0](https://github.com/utiasDSL/crisp_py/compare/v1.2.3...v1.3.0) (2025-08-10)


### Features

* include data files to the repo ([1b4d125](https://github.com/utiasDSL/crisp_py/commit/1b4d1252ef074172dca93e6cca99e017d6dbf745))


### Bug Fixes

* deprecated license option in pyproject.toml ([94f18b8](https://github.com/utiasDSL/crisp_py/commit/94f18b8fdae265f5fc08b1671adf1cf602feeb15))

## [1.2.3](https://github.com/utiasDSL/crisp_py/compare/v1.2.2...v1.2.3) (2025-08-10)


### Bug Fixes

* add license + homepage ([05cd2e7](https://github.com/utiasDSL/crisp_py/commit/05cd2e72a0cc481e94645f61a1644aa41518a1c2))
* added proper versioning for package ([5d49611](https://github.com/utiasDSL/crisp_py/commit/5d4961170aa0915af3f7e1be1df0d085bb1f0713))
* Update README.md to be able to visualize banner in pypi ([42e82a4](https://github.com/utiasDSL/crisp_py/commit/42e82a4beaa4816682d0b0418a49bc541bf053f4))
* wrong version in __init__ ([76bc4cf](https://github.com/utiasDSL/crisp_py/commit/76bc4cf9503a7345caacede136e78485a9aaf48f))

## [1.2.2](https://github.com/utiasDSL/crisp_py/compare/v1.2.1...v1.2.2) (2025-08-10)


### Bug Fixes

* guide use in case that the ros2 control packages are not installed ([19a616d](https://github.com/utiasDSL/crisp_py/commit/19a616d0d0cfad794df171c2ee14122ba80f0556))

## [1.2.1](https://github.com/utiasDSL/crisp_py/compare/v1.2.0...v1.2.1) (2025-08-10)


### Bug Fixes

* New name for pypi, since crisp-py not allowed ([6dfee56](https://github.com/utiasDSL/crisp_py/commit/6dfee5628eda102711bc27c261fd6014a2aa4d26))

## [1.2.0](https://github.com/utiasDSL/crisp_py/compare/v1.1.2...v1.2.0) (2025-08-10)


### Features

* added number of joint to robot_config ([d921f3d](https://github.com/utiasDSL/crisp_py/commit/d921f3db438e57fffe80d134bbea4a41667a8b54))
* added release bot ([5a02e1b](https://github.com/utiasDSL/crisp_py/commit/5a02e1b1b087c9f24c46d6ee1e466eac2a2aa0d1))
* modified release.yaml to upload directly to test-pypi ([166fba0](https://github.com/utiasDSL/crisp_py/commit/166fba026b3e8b58577627e472ee465e1473c2c4))
* reverting to version 1.0.0 for first official package ([c18220a](https://github.com/utiasDSL/crisp_py/commit/c18220a997a9990fe502ff3c22b2e8a8fa327ecf))


### Bug Fixes

* index for gripper is now part of the gripper config instead of gripper ([8079619](https://github.com/utiasDSL/crisp_py/commit/807961976ad29f70d358d03987f3185a8b7c7fd4))
* specifying the location of the version.txt ([633b5fc](https://github.com/utiasDSL/crisp_py/commit/633b5fc96dd1a08fddb764bee3f72234f08c1a4c))
* test for sensor ([ad9a6fb](https://github.com/utiasDSL/crisp_py/commit/ad9a6fba054e98afc3cbf77ca97a5efd9dc543ae))
* updated release-please configs ([5b9210d](https://github.com/utiasDSL/crisp_py/commit/5b9210dc172beb269f2276c54f112dbdd3b35377))
* use python release type instead of simple ([2c9fe29](https://github.com/utiasDSL/crisp_py/commit/2c9fe294ac74bdb30175862c3677046cf3f0cac3))
* version.txt and tests should not be packaged ([43a9ca7](https://github.com/utiasDSL/crisp_py/commit/43a9ca70dbbd11c458054fb1412b6e013a284f35))
* versioning of the package ([29d568e](https://github.com/utiasDSL/crisp_py/commit/29d568eb47e95787e043270177471c664ce06c5d))
