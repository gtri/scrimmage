#ifndef INCLUDE_SCRIMMAGE_GPU_GPUBUFFER_H
#define INCLUDE_SCRIMMAGE_GPU_GPUBUFFER_H

#include <scrimmage/gpu/OpenCLUtils.h>

#if ENABLE_GPU_ACCELERATION == 1
#include <CL/opencl.hpp>
#endif

#include <cassert>
#include <iterator>
#include <optional>

namespace scrimmage {

/*
 * Helps in memory managemnt between host and gpu
 *
 */

template <typename T>
class GPUMapBuffer {
 public:
  typedef T* iterator;
  typedef T const* const_iterator;

  iterator begin() { return iterator(&host_buffer_[0]); };
  iterator end() { return iterator(&host_buffer_[size_]); };

  const_iterator cbegin() { return const_iterator(&host_buffer_[0]); };
  const_iterator cend() { return const_iterator(&host_buffer_[size_]); };

  GPUMapBuffer(cl::CommandQueue queue, std::size_t num_elements, cl_mem_flags buffer_flags)
      : queue_{queue},
        context_{queue.getInfo<CL_QUEUE_CONTEXT>()},
        device_{queue.getInfo<CL_QUEUE_DEVICE>()},
        size_{num_elements},
        host_buffer_{nullptr},
        buffer_flags_{buffer_flags | CL_MEM_ALLOC_HOST_PTR}
  // Tell OpenCL we want to use the ptrs it allocates
  {
    std::optional<std::size_t> cacheline_size_opt =
        OpenCLUtils::cacheline_size(device_);  // This should be for individual devices
    if (cacheline_size_opt.has_value()) {
      buffer_alignment_ = cacheline_size_opt.value();
    } else {
      buffer_alignment_ = DEFAULT_BUFFER_ALIGNMENT;
    }
    capacity_ = next_largest_alignment_size(num_elements * sizeof(T));
  }

  GPUMapBuffer(cl::CommandQueue queue, cl_mem_flags buffer_flags)
      : GPUMapBuffer{queue, 0, buffer_flags} {}

  ~GPUMapBuffer() { _unmap_buffer(); }

  std::size_t device_buffer_size() { return device_buffer_.getInfo<CL_MEM_SIZE>(); }

  void push_back(T element) {
    if (size_ == capacity_ * sizeof(T)) {
      throw std::out_of_range{
          "No more room left in buffer to add element. Try resizing buffer to correct size."};
    }
    host_buffer_[size_++] = element;
  }

  // Returns iterator to pos after insertion
  template <typename InputIterator>
  iterator insert(const_iterator pos, InputIterator start, InputIterator end) {
    std::ptrdiff_t offset = pos - cbegin();
    std::ptrdiff_t num_additional = std::distance(start, end);
    if (sizeof(T) * (num_additional + size_) > capacity_) {
      throw std::out_of_range{
          "Tried to add more elements to MapBuffer than MapBuffer has capacity for."};
    }

    for (auto it = start; it != end; ++it) {
      std::size_t index = offset + std::distance(start, it);
      host_buffer_[index + num_additional] = host_buffer_[index];
      host_buffer_[index] = static_cast<T>(*it);
    }
    size_ += num_additional;
    return iterator(&host_buffer_[offset + num_additional]);
  }

  template <typename E>
  iterator insert(const_iterator pos, std::initializer_list<E> to_insert) {
    return insert(pos, to_insert.begin(), to_insert.end());
  }

  void resize(std::size_t n) {
    std::size_t byte_size = n * sizeof(T);
    capacity_ = next_largest_alignment_size(byte_size);
  }

  bool map(cl_mem_flags map_flags) {
    bool success = true;
    // Check that underlying memory object exists for device buffer
    if (host_buffer_ != nullptr) {
      std::cerr << "GPUMapBuffer is already mapped into memory, and will not be remapped. Did you "
                   "forget to unmap the buffer?\n";
      success = false;
    } else if (device_buffer_.get() == nullptr ||
               device_buffer_.getInfo<CL_MEM_SIZE>() != capacity_) {
      success &= _allocate_buffer() == CL_SUCCESS;
    }
    success &= _map_buffer(map_flags) == CL_SUCCESS;
    return success;
  };

  bool unmap() { return _unmap_buffer() == CL_SUCCESS; };

  // Returns the pointer to the underlying host_buffer_
  const T* host_buffer() const { return host_buffer(); }

  // Returns the pointer to the underlying host_buffer_
  T* host_buffer() { return host_buffer_; }

  // Returns the underlying device buffer
  const cl::Buffer& device_buffer() const { return device_buffer_; }

  // Returns the underlying device buffer
  cl::Buffer& device_buffer() { return device_buffer_; }

  T& at(std::size_t index) {
    if (host_buffer_ == nullptr || index >= capacity_) {
      throw std::out_of_range{
          "GPUMapBuffer either has not been mapped or trying to access memory outside its range."};
    }
    return host_buffer_[index];
  }

  const T& at(std::size_t index) const { return at(index); }

  T& operator[](std::size_t index) { return host_buffer_[index]; }

  const T& operator[](std::size_t index) const { return host_buffer_[index]; }

  std::size_t size() { return size_; }
  std::size_t capacity() { return capacity_; }

  static constexpr bool BLOCKING_MAP = true;
  static constexpr std::size_t OFFSET = 0;
  static constexpr std::size_t DEFAULT_BUFFER_ALIGNMENT = 64;

 private:
  cl::CommandQueue queue_;
  cl::Context context_;
  cl::Device device_;
  std::size_t size_;
  std::size_t capacity_;
  cl::Buffer device_buffer_;

  T* host_buffer_;
  size_t buffer_alignment_;
  const cl_mem_flags buffer_flags_;

  cl_int _allocate_buffer() {
    cl_int err = CL_SUCCESS;
    device_buffer_ = cl::Buffer{context_, buffer_flags_, capacity_, nullptr, &err};
    return err;
  }

  cl_int _map_buffer(cl_mem_flags map_flags) {
    cl_int err = CL_SUCCESS;
    host_buffer_ = static_cast<T*>(queue_.enqueueMapBuffer(
        device_buffer_, BLOCKING_MAP, map_flags, OFFSET, capacity_, nullptr, nullptr, &err));
    assert(host_buffer_ != nullptr);
    return err;
  }

  cl_int _unmap_buffer() {
    cl_int err = CL_SUCCESS;
    if (host_buffer_ != nullptr) {
      err = queue_.enqueueUnmapMemObject(device_buffer_, host_buffer_);
      host_buffer_ = nullptr;
      size_ = 0;
    }
    return err;
  }

  // Returns the next largest value that is a multiple of 64 bytes
  // to make our buffers aligned with cacheline size
  std::size_t next_largest_alignment_size(std::size_t bytes) {
    if (bytes == 0) {
      return buffer_alignment_;
    }
    return (1 + ((bytes - 1) / buffer_alignment_)) * buffer_alignment_;
  }
};
}  // namespace scrimmage

#endif  // INCLUDE_SCRIMMAGE_GPU_GPUBUFFER_H
