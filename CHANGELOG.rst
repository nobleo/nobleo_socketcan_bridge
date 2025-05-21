.. Copyright (C) 2025 Nobleo Technology B.V.
..
.. SPDX-License-Identifier: Apache-2.0

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nobleo_socketcan_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Remove deprecated call to ament_target_dependencies
* Bump minimum CMake version to avoid compatiility warning
* Contributors: Ramon Wijnands

1.0.2 (2025-04-02)
------------------
* fix: make `state_` atomic
  It is accessed from the main thread and updated via the receiver thread.
* Add url to the package.xml
* Fix CI by updating its dependencies
* Contributors: Ramon Wijnands, Rein Appeldoorn

1.0.1 (2025-02-28)
------------------
* Disable clang_format on humble
  This version of clang_format has a different output.
* Contributors: Ramon Wijnands

1.0.0 (2025-02-12)
------------------
* First release
* Contributors: Ramon Wijnands, dkorenkovnl, Rein Appeldoorn, Ferry Schoenmakers, Tim Clephas
