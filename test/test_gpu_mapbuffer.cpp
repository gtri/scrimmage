/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Wesley Ford <wford32@gatech.edu>
 * @date 23 April 2024
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <gtest/gtest.h>

#include <scrimmage/gpu/GPUController.h>
#include <scrimmage/gpu/GPUMapBuffer.h>

#include <CL/opencl.hpp>

#include <vector>
#include <numeric>
#include <typeinfo>

namespace sc = scrimmage;
using namespace testing;

template<typename T>
class GPUTestFixture : public Test {
  protected:
    GPUTestFixture()
    {
      gpu = std::make_shared<sc::GPUController>();
      gpu->init();
    }

  public:
    std::shared_ptr<sc::GPUController> gpu;
};

using TestingTypes = ::testing::Types<char, int, long, float, double>;
TYPED_TEST_SUITE(GPUTestFixture, TestingTypes);


TYPED_TEST(GPUTestFixture, TestZeroAllocAlignment) {
  sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, 0,
    CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};

  // Zero elements allocated in constructure should result in the 
  // default buffer alignment
  ASSERT_EQ(map_buffer.capacity(), 
      sc::GPUMapBuffer<TypeParam>::DEFAULT_BUFFER_ALIGNMENT);
}

TYPED_TEST(GPUTestFixture, TestNonZeroAllocAlignment) {
  std::size_t max_capacity = 1<<30;
  std::size_t buff_align = sc::GPUMapBuffer<TypeParam>::DEFAULT_BUFFER_ALIGNMENT;
  for(std::size_t expected_capacity = buff_align; expected_capacity <= max_capacity; expected_capacity *= buff_align) {
    std::size_t num_elements = expected_capacity / sizeof(TypeParam);
    std::cout << "Num elements " << num_elements << std::endl;
    sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, num_elements ,
      CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};
    ASSERT_EQ(map_buffer.size(), num_elements);
    ASSERT_EQ(map_buffer.capacity(), expected_capacity);
  }
}

TYPED_TEST(GPUTestFixture, TestBufferReadWrite) {
  std::size_t num_elements = 1<<10;
  sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, num_elements,
    CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};
  
  map_buffer.map(CL_MAP_READ | CL_MAP_WRITE);
  for(std::size_t i = 0; i < num_elements; i++) {
    map_buffer[i] = static_cast<TypeParam>(i); 
  }

  for(std::size_t i = 0; i < num_elements; i++) {
    ASSERT_EQ(map_buffer[i], static_cast<TypeParam>(i)); 
  }
  map_buffer.unmap();
}

TYPED_TEST(GPUTestFixture, TestEndInsert) {
  std::vector<TypeParam> elements(4000);
  std::iota(elements.begin(), elements.end(), 0);
  
  // Make sure that there are no elements in the buffer
  sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, 0,
    CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};

  map_buffer.resize(elements.size()); 
  map_buffer.map(CL_MAP_READ | CL_MAP_WRITE);
  ASSERT_EQ(map_buffer.size(), 0);
  ASSERT_GE(map_buffer.capacity(), elements.size()*sizeof(TypeParam));
  
  // Test End Assertion
  auto end_it = map_buffer.insert(map_buffer.end(), elements.begin(), elements.end());

  ASSERT_EQ(map_buffer.size(), elements.size());
  ASSERT_EQ(end_it, map_buffer.end());
  ASSERT_GE(map_buffer.capacity(), elements.size()*sizeof(TypeParam));

  for(std::size_t i = 0; i < map_buffer.size(); ++i) {
    ASSERT_EQ(map_buffer.at(i), elements.at(i));
  }

  map_buffer.unmap();
}

TYPED_TEST(GPUTestFixture, TestFrontInsert) {
  std::size_t total_elements = 4000;
  std::vector<TypeParam> elements(total_elements / 2);
  std::iota(elements.begin(), elements.end(), 0);
  
  // Make sure that there are no elements in the buffer
  sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, 0,
    CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};

  map_buffer.resize(total_elements); 
  map_buffer.map(CL_MAP_READ | CL_MAP_WRITE);
 
  ASSERT_EQ(map_buffer.size(), 0);
  ASSERT_GE(map_buffer.capacity(), elements.size()*sizeof(TypeParam));
  
  // Insert half elements at the beginning 
  map_buffer.insert(map_buffer.begin(), elements.begin(), elements.end());

  ASSERT_EQ(map_buffer.size(), elements.size());
  ASSERT_GE(map_buffer.capacity(), total_elements*sizeof(TypeParam));

  // Insert elements again. In reverse order 
  map_buffer.insert(map_buffer.begin(), elements.rbegin(), elements.rend());
  ASSERT_EQ(map_buffer.size(), total_elements);
  ASSERT_GE(map_buffer.capacity(), total_elements*sizeof(TypeParam));

  auto buff_it = map_buffer.begin();
  for(auto it = elements.rbegin(); it != elements.rend(); ++it) {
    ASSERT_EQ(*buff_it, *it);
    ++buff_it;
  }
  for(auto it = elements.begin(); it != elements.end(); ++it) {
    ASSERT_EQ(*buff_it, *it);
    ++buff_it;
  }
  map_buffer.unmap();
}

TYPED_TEST(GPUTestFixture, TestMiddleInsert) {
  std::size_t total_elements = 3000;
  std::vector<TypeParam> elements(total_elements / 3);
  std::iota(elements.begin(), elements.end(), 0);

  // Make sure that there are no elements in the buffer
  sc::GPUMapBuffer<TypeParam> map_buffer{this->gpu, 0,
    CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR};

  map_buffer.resize(total_elements); 
  map_buffer.map(CL_MAP_READ | CL_MAP_WRITE);
 
  ASSERT_EQ(map_buffer.size(), 0);
  ASSERT_GE(map_buffer.capacity(), elements.size()*sizeof(TypeParam));
  
  // Insert half elements at the beginning 
  auto middle_insert = map_buffer.insert(map_buffer.end(), elements.begin(), elements.end());
  map_buffer.insert(map_buffer.end(), elements.begin(), elements.end());
  map_buffer.insert(middle_insert, elements.begin(), elements.end());

  ASSERT_EQ(map_buffer.size(), total_elements);
  ASSERT_GE(map_buffer.capacity(), total_elements*sizeof(TypeParam));

  for(auto it = elements.begin(); it != elements.end(); ++it) {
    std::size_t index = std::distance(elements.begin(), it);
    ASSERT_EQ(map_buffer.at(index), *it);
    ASSERT_EQ(map_buffer.at(elements.size() + index), *it);
    ASSERT_EQ(map_buffer.at(2*elements.size() + index), *it);
  }

  map_buffer.unmap();
}
