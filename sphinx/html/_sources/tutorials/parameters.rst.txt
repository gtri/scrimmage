.. _parameters:

Plugin Parameters
=========================

Each SCRIMMAGE plugin is parameterized by the parameters in its respective
PluginName.xml file. For example, the ``Straight.xml`` file has a parameter,
``speed``, that specifies the desired forward speed for the entity. During
plugin initialization, this parameter can be parsed and saved to a member variable of
the Straight class with the help of SCRIMMAGE's ``get()`` function:

.. code-block:: c++
   :linenos:

   void Straight::init(std::map<std::string, std::string> &params) {
       speed_ = scrimmage::get<double>("speed", params, 0.0);
   ...

The previous method for setting plugin parameters only applies during plugin
initialization. If you want to update a parameter during runtime, you can use
the ``register_param()`` function. For example, if you wanted to update a
member variable, ``desired_z_``, during runtime, you can register this
parameter in your plugin's ``init()`` function:

.. code-block:: c++
   :linenos:
      
   register_param<double>("desired_z", desired_z_);

During the simulation, another plugin can call the ``set_param()`` function:

.. code-block:: c++
   :linenos:

   set_param<double>("desired_z", 100.0)

while will have the affect of directly modifying the value of ``desired_z_`` in
the plugin that registered that parameter. If you require a callback function
when the parameter value is updated, you can use the optional callback form of
the ``register_param()`` function:
   
.. code-block:: c++
   :linenos:
      
   auto param_cb = [&](const double &desired_z) {
       std::cout << "desired_z param changed to: " << desired_z << endl;
   };
   register_param<double>("desired_z", desired_z_, param_cb);

You can also "unregister" the parameter update with the ``unregister_param``
function:

.. code-block:: c++
   :linenos:

   unregister_param<double>("desired_z");
