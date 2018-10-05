Change Log
==========

2018-10

* plugins inheriting from ``ScrimmageOpenAIAutonomy`` now override ``init_helper``
  and ``step_helper`` instead of ``init`` and ``step_autonomy``.
* To access python libraries, do ``import scrimmage.utils``, ``import
  scrimmage.bindings``, and ``import scrimmaage.proto_utils``
