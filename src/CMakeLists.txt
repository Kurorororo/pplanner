add_library (sas_plus STATIC
  sas_plus.cc
  ./sas_plus/partial_state.cc
  ./sas_plus/partial_state_vector.cc
  ./sas_plus/effect_vector.cc ./sas_plus/facts.cc
  ./sas_plus/mutex_groups.cc
  ./sas_plus/parse_utils.cc
  ./sas_plus/strong_stubborn_sets.cc)

add_library(search_graph STATIC
  ./search_graph.cc
  ./search_graph_factory.cc
  ./distributed_search_graph_factory.cc
  ./search_graph/distributed_search_graph_with_landmarks.cc
  ./search_graph/distributed_search_graph.cc
  ./search_graph/state_packer.cc
  ./hash/distribution_hash.cc
  ./hash/distribution_hash_factory.cc
  ./hash/gra_zobrist_hash.cc
  ./hash/zobrist_hash.cc)

add_library(evaluators STATIC
  evaluator.cc
  evaluator_factory.cc
  random_walk_evaluator.cc
  random_walk_evaluator_factory.cc
  ./heuristics/blind.cc
  ./heuristics/new_operator.cc
  ./heuristics/width.cc)

add_library(search STATIC
  search.cc
  search_factory.cc
  ./search/kgbfs.cc
  ./search/gbfs.cc
  ./search/dehc.cc
  ./search/lazy_gbfs.cc
  ./search/orbit_gbfs.cc
  ./search/symmetry_breaking_gbfs.cc
  ./search/mrw13.cc
  ./search/multi_focus_gbfs.cc
  ./search/simhdgbfs.cc
  ./search/simhdgbfs1.cc
  ./symmetry/symmetry.cc
  ./utils/lexical_order.cc
  ./multithread_search/abstract_graph.cc
  ./multithread_search/search_node.cc
  ./multithread_search/closed_list.cc
  ./multithread_search/lock_free_closed_list.cc
  ./multithread_search/gbfs_portfolio.cc
  ./multithread_search/multi_gbfs.cc
  ./multithread_search/multi_focus_gbfs.cc
  ./multithread_search/mcs_independent.cc
  ./multithread_search/greedy_pbnf.cc)

add_library(ff STATIC
  ./heuristics/rpg.cc
  ./heuristics/hn_rpg.cc
  ./heuristics/rpg_table.cc
  ./heuristics/relaxed_sas_plus.cc)

add_library(landmark STATIC
  dtg.cc
  ./landmark/landmark.cc
  ./landmark/landmark_count_base.cc
  ./landmark/landmark_detection.cc
  ./landmark/landmark_graph.cc
  ./landmark/generating_orderings.cc)

add_library(dominance STATIC
  ./dominance/lds.cc
  ./dominance/qdf.cc
  ./dominance/lts.cc)
