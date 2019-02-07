.. _utilities:

Useful Utilities
================

Python Package
--------------

The following utilities require that you have installed the scrimmage python
package. See the `instructions`_ in the main README.md to install a developer
version of the scrimmage python bindings.

Convert frames.bin to CSV File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can create a CSV file (e.g., ``frames.csv``) from a ``frames.bin`` file
with the following code:

.. code-block:: python
   :linenos:

   import scrimmage.utils as su
   su.frames2csv('/home/youruser/.scrimmage/logs/latest/frames.bin')

The frames can be downsampled with the ``new_dt`` argument and the output file
name can be changed with the ``out_filename`` argument.

.. code-block:: python
   :linenos:

   su.frames2csv('/home/youruser/.scrimmage/logs/latest/frames.bin', new_dt=15.0, out_filename='fewer_frames.bin')

.. _instructions: https://github.com/gtri/scrimmage#install-scrimmage-python-bindings
