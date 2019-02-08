cmake_minimum_required (VERSION 3.8)
project(pplanner)

if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set (CMAKE_BUILD_TYPE "Release")
endif ()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "-pthread -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
enable_testing()

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED OFF)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package (Boost 1.63.0 COMPONENTS program_options system)

set(GTEST_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/googletest/googletest/)

include_directories(./src
                    ./lib/bliss
                    ${Boost_INCLUDE_DIRS}
                    ${GTEST_ROOT}/include)
link_directories(./lib/bliss ${GTEST_ROOT})

add_subdirectory(src)
add_subdirectory(${GTEST_ROOT})

find_package(MPI)
find_package(CUDA 8.0)

# MPI planner
if (MPI_CXX_FOUND)
  add_executable(mpi_planner
    ./src/mpi_planner.cc
    ./src/mpi_search_factory.cc
    ./src/mpi_search/pddsgbfs.cc
    ./src/mpi_search/pigbfs.cc
    ./src/mpi_search/hddehc.cc
    ./src/mpi_search/hdgbfs.cc
    ./src/mpi_search/hdgbfs1.cc
    ./src/mpi_search/symmetry_breaking_pddsgbfs.cc
    ./src/mpi_search/symmetry_breaking_hdgbfs.cc
    ./src/successor_generator.cc
    ./src/postprocess/action_elimination.cc
    ./src/utils/file_utils.cc)
  target_include_directories(mpi_planner PRIVATE ${MPI_CXX_INCLUDE_PATH})
  target_compile_options(mpi_planner PRIVATE
    ${MPI_CXX_COMPILE_FLAGS} "-Wall" "-O3" "-DNDEBG")
  target_link_libraries(mpi_planner
    search
    evaluators
    landmark
    ff
    search_graph
    dominance
    sas_plus
    bliss
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    ${MPI_CXX_LIBRARIES}
    ${MPI_CXX_LINK_FLAGS})
endif ()

# CUDA planner
if (CUDA_FOUND)
  set(CUDA_SEPARABLE_COMPILATION ON)
  set(CUDA_PROPAGATE_HOST_FLAGS ON)
  set(CUDA_NVCC_FLAGS "-std=c++14" "-arch=compute_61" "-code=sm_61")

  string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPER)

  if (CMAKE_BUILD_TYPE_UPPER STREQUAL "RELEASE")
    set(CUDA_NVCC_FLAGS_RELEASE -O3;-DNDEBG)
  elseif (CMAKE_BUILD_TYPE_UPPER STREQUAL "RELWITHDEBINFO")
    set(CUDA_NVCC_FLAGS_RELWITHDEBINFO -O3;-lineinfo;-Xcompiler;-rdynamic)
  elseif (CMAKE_BUILD_TYPE_UPPER STREQUAL "DEBUG")
    set(CUDA_NVCC_FLAGS_DEBUG -O3;-g;-G)
  endIf ()

  cuda_add_executable(cuda_planner
    ./src/cuda_planner.cu
    ./src/cuda_sas_plus.cu
    ./src/cuda_successor_generator.cu
    ./src/cuda_search_graph.cu
    ./src/cuda_search_factory.cu
    ./src/cuda_common/cuda_random.cu
    ./src/cuda_search/cuda_bmrw.cu
    ./src/cuda_search/cuda_bmrw_gbfs.cu
    ./src/cuda_search/cuda_bmrw_gbfs_share.cu
    ./src/cuda_search/cuda_bmrw_gbfs_focus.cu
    ./src/cuda_search/cuda_bmrw_gbfs_focus2.cu
    ./src/cuda_search/cuda_hetro_gbfs.cu
    ./src/cuda_search/cuda_hdgbfs.cu
    ./src/cuda_search/cuda_rwagbfs.cu
    ./src/cuda_search/cuda_gbfs_kernel.cu
    ./src/cuda_search/cuda_random_walk.cu
    ./src/cuda_landmark/cuda_landmark_graph.cu
    ./src/cuda_landmark/cuda_landmark_count_base.cu
    ./src/cuda_hash/cuda_zobrist_hash.cu
    ./src/successor_generator.cc
    ./src/postprocess/action_elimination.cc
    ./src/utils/file_utils.cc)
  target_link_libraries(cuda_planner
    search
    evaluators
    landmark
    ff
    search_graph
    sas_plus
    ${Boost_PROGRAM_OPTIONS_LIBRARY})

  cuda_add_executable(time_cuda_prefix_sum
    ./tests/cuda_common/time_cuda_prefix_sum.cu
    ./src/cuda_common/cuda_prefix_sum.cu)

  cuda_add_executable(test_cuda_prefix_sum
    ./tests/cuda_common/test_cuda_prefix_sum.cu
    ./src/cuda_common/cuda_prefix_sum.cu)
  add_test(NAME CudaPrefixSumTest COMMAND test_cuda_prefix_sum)

  cuda_add_executable(test_cuda_sas_plus
    ./tests/test_cuda_sas_plus.cu ./src/cuda_sas_plus.cu)
  add_test(NAME CudaSASPlusTest COMMAND test_cuda_sas_plus)
  target_link_libraries(test_cuda_sas_plus sas_plus)

  cuda_add_executable(test_cuda_successor_generator
    ./tests/test_cuda_successor_generator.cu
    ./src/cuda_successor_generator.cu
    ./src/cuda_sas_plus.cu
    ./src/successor_generator.cc
    ./src/cuda_common/cuda_random.cu)
  add_test(
    NAME CudaSuccessorGeneratorTest COMMAND test_cuda_successor_generator)
  target_link_libraries(test_cuda_successor_generator sas_plus)
endif()

# sequential planner
add_executable(planner
  ./src/planner.cc
  ./src/successor_generator.cc
  ./src/postprocess/action_elimination.cc
  ./src/utils/file_utils.cc)
target_link_libraries(planner
  search
  evaluators
  landmark
  ff
  search_graph
  dominance
  sas_plus
  bliss
  ${Boost_PROGRAM_OPTIONS_LIBRARY})

# execute relaxed graphplan from SAS+
add_executable(sas_relaxed_graphplan
  ./tests/heuristics/sas_relaxed_graphplan.cc
  ./src/heuristics/hn_rpg.cc
  ./src/heuristics/rpg.cc
  ./src/heuristics/relaxed_sas_plus.cc
  ./src/utils/file_utils.cc)
target_link_libraries(sas_relaxed_graphplan
  sas_plus ${Boost_PROGRAM_OPTIONS_LIBRARY})

# SAS+ parser
add_executable(sas_parser ./tests/sas_parser.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_parser sas_plus)

# dump DTGs (Domain Transision Graphs) from SAS+
add_executable(sas_dtg
  ./tests/sas_dtg.cc ./src/dtg.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_dtg sas_plus)

# detect landmarks from SAS+
add_executable(sas_landmark_detection
  ./tests/landmark/sas_landmark_detection.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_landmark_detection landmark ff sas_plus)

# dump LTSs (Labeled Transition Systems) from SAS+
add_executable(sas_lts ./tests/sas_lts.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_lts dominance sas_plus)

# dump LDSs (Labeled Dominance Systems) from SAS+
add_executable(sas_lds ./tests/sas_lds.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_lds dominance sas_plus)

# dump QDF (Quantitive Dominance Function) from SAS+
add_executable(sas_qdf ./tests/sas_qdf.cc ./src/utils/file_utils.cc)
target_link_libraries(sas_qdf dominance sas_plus)

# test codes
add_executable(test_partial_state
  ./tests/sas_plus/test_partial_state.cc ./src/sas_plus/partial_state.cc)
target_link_libraries(test_partial_state gtest gtest_main pthread)
add_test(NAME PartialStateTest COMMAND test_partial_state)

add_executable(test_partial_state_vector
  ./tests/sas_plus/test_partial_state_vector.cc
  ./src/sas_plus/partial_state_vector.cc)
target_link_libraries(test_partial_state_vector gtest gtest_main pthread)
add_test(NAME PartialStateVectorTest COMMAND test_partial_state_vector)

add_executable(test_effect_vector
  ./tests/sas_plus/test_effect_vector.cc
  ./src//sas_plus/effect_vector.cc
  ./src//sas_plus/partial_state_vector.cc)
target_link_libraries(test_effect_vector gtest gtest_main pthread)
add_test(NAME EffectVectorTest COMMAND test_effect_vector)

add_executable(test_facts
  ./tests/sas_plus/test_facts.cc ./src/sas_plus/facts.cc)
target_link_libraries(test_facts gtest gtest_main pthread)
add_test(NAME FactsTest COMMAND test_facts)

add_executable(test_mutex_groups
  ./tests/sas_plus/test_mutex_groups.cc ./src/sas_plus/mutex_groups.cc)
target_link_libraries(test_mutex_groups gtest gtest_main pthread)
add_test(NAME MutexGroupsTest COMMAND test_mutex_groups)

add_executable(test_sas_plus ./tests/test_sas_plus.cc)
target_link_libraries(test_sas_plus sas_plus gtest gtest_main pthread)
add_test(NAME SASPlusTest COMMAND test_sas_plus)

add_executable(test_successor_generator
  ./tests/test_successor_generator.cc ./src/successor_generator.cc)
target_link_libraries(test_successor_generator
  sas_plus gtest gtest_main pthread)

add_executable(test_zobrist_hash
  ./tests/hash/test_zobrist_hash.cc
  ./src/hash/distribution_hash.cc
  ./src/hash/zobrist_hash.cc)
target_link_libraries(test_zobrist_hash sas_plus gtest gtest_main pthread)
add_test(NAME ZobristHashTest COMMAND test_zobrist_hash)

add_executable(test_pair_hash ./tests/hash/test_pair_hash.cc)
target_link_libraries(test_pair_hash gtest gtest_main pthread)
add_test(NAME PairHashTest COMMAND test_pair_hash)

add_executable(test_array_hash ./tests/hash/test_array_hash.cc)
target_link_libraries(test_array_hash gtest gtest_main pthread)
add_test(NAME ArrayHashTest COMMAND test_array_hash)

add_executable(test_state_packer
  ./tests/search_graph/test_state_packer.cc ./src/search_graph/state_packer.cc)
target_link_libraries(test_state_packer sas_plus gtest gtest_main pthread)
add_test(NAME StatePackerTest COMMAND test_state_packer)

add_executable(test_search_graph ./tests/test_search_graph.cc)
target_link_libraries(test_search_graph
  sas_plus search_graph gtest gtest_main pthread)
add_test(NAME SearchGraphTest COMMAND test_search_graph)

add_executable(test_blind
  ./tests/heuristics/test_blind.cc
  ./src/heuristics/blind.cc
  ./src/evaluator.cc)
target_link_libraries(test_blind sas_plus gtest gtest_main pthread)
add_test(NAME BlindTest COMMAND test_blind)

add_executable(test_relaxed_sas_plus
  ./tests/heuristics/test_relaxed_sas_plus.cc
  ./src/heuristics/relaxed_sas_plus.cc)
target_link_libraries(test_relaxed_sas_plus sas_plus gtest gtest_main pthread)
add_test(NAME RelaxedSASPlusTest COMMAND test_relaxed_sas_plus)

add_executable(test_rpg_table
  ./tests/heuristics/test_rpg_table.cc
  ./src/heuristics/rpg.cc
  ./src/heuristics/rpg_table.cc
  ./src/heuristics/relaxed_sas_plus.cc)
target_link_libraries(test_rpg_table sas_plus gtest gtest_main pthread)
add_test(NAME RPGTableTest COMMAND test_rpg_table)

add_executable(test_additive
  ./tests/heuristics/test_additive.cc
  ./src/evaluator.cc
  ./src/heuristics/rpg.cc
  ./src/heuristics/rpg_table.cc
  ./src/heuristics/relaxed_sas_plus.cc)
target_link_libraries(test_additive sas_plus gtest gtest_main pthread)
add_test(NAME AdditiveTest COMMAND test_additive)

add_executable(test_ff_add
  ./tests/heuristics/test_ff_add.cc
  ./src/evaluator.cc
  ./src/heuristics/rpg.cc
  ./src/heuristics/rpg_table.cc
  ./src/heuristics/relaxed_sas_plus.cc)
target_link_libraries(test_ff_add sas_plus gtest gtest_main pthread)
add_test(NAME FFAddTest COMMAND test_ff_add)

add_executable(test_fifo_open_list_impl
  ./tests/open_lists/test_fifo_open_list_impl.cc)
target_link_libraries(test_fifo_open_list_impl gtest gtest_main pthread)
add_test(NAME FIFOOpenListImplTest COMMAND test_fifo_open_list_impl)

add_executable(test_open_list_impl_factory
  ./tests/open_lists/test_open_list_impl_factory.cc)
target_link_libraries(test_open_list_impl_factory gtest gtest_main pthread)
add_test(NAME OpenListImplFactoryTest COMMAND test_open_list_impl_factory)

add_executable(test_single_open_list
  ./tests/open_lists/test_single_open_list.cc
  ./src/heuristics/blind.cc
  ./src/evaluator.cc)
target_link_libraries(test_single_open_list
  sas_plus search_graph gtest gtest_main pthread)
add_test(NAME SingleOpenListTest COMMAND test_single_open_list)

add_executable(test_preferred_open_list
  ./tests/open_lists/test_preferred_open_list.cc
  ./src/heuristics/blind.cc
  ./src/evaluator.cc)
target_link_libraries(test_preferred_open_list
  sas_plus search_graph gtest gtest_main pthread)
add_test(NAME PreferredOpenListTest COMMAND test_preferred_open_list)

add_executable(test_landmark
  ./tests/landmark/test_landmark.cc ./src/landmark/landmark.cc)
target_link_libraries(test_landmark gtest gtest_main pthread)
add_test(NAME LandmarkTest COMMAND test_landmark)

add_executable(test_landmark_graph
  ./tests/landmark/test_landmark_graph.cc
  ./src/landmark/landmark_graph.cc
  ./src/landmark/landmark.cc)
target_link_libraries(test_landmark_graph sas_plus gtest gtest_main pthread)
add_test(NAME LandmarkGraphTest COMMAND test_landmark_graph)

add_executable(test_dtg tests/test_dtg.cc ./src/dtg.cc)
target_link_libraries(test_dtg sas_plus gtest gtest_main pthread)
add_test(NAME DTGTest COMMAND test_dtg)

add_executable(test_lts tests/dominance/test_lts.cc ./src/dominance/lts.cc)
target_link_libraries(test_lts sas_plus gtest gtest_main pthread)
add_test(NAME LTSTest COMMAND test_lts)

