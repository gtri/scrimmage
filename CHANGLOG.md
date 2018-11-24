Change Log
==========

2018-11
* ``calc_reward`` in ``ScrimmageOpenAIAutonomy`` now return a tuple of whether
  the plugin is done, the reward, and a pybind11 dictionary. The prior behavior
  was a pair including done and the reward.

2018-10

* plugins inheriting from ``ScrimmageOpenAIAutonomy`` now override ``init_helper``
  and ``step_helper`` instead of ``init`` and ``step_autonomy``.
* To access python libraries, do ``import scrimmage.utils``, ``import
  scrimmage.bindings``, and ``import scrimmaage.proto_utils``
