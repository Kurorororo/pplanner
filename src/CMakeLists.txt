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
  ./closed_list.cc
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
  ./heuristics/blind.cc
  ./heuristics/new_operator.cc
  ./heuristics/width.cc)

add_library(search STATIC
  closed_list.cc
  search.cc
  search_factory.cc
  search_node.cc
  ./search/kgbfs.cc
  ./search/gbfs.cc
  ./search/bts_gbfs.cc
  ./search/dehc.cc
  ./search/lazy_gbfs.cc
  ./search/orbit_gbfs.cc
  ./search/symmetry_breaking_gbfs.cc
  ./search/mrw13.cc
  ./search/bmrw.cc
  ./search/multi_focus_gbfs.cc
  ./search/simhdgbfs.cc
  ./search/simhdgbfs1.cc
  ./symmetry/symmetry.cc
  ./utils/lexical_order.cc
  ./sas_plus/abstract_graph.cc
  ./multithread_search/lock_free_closed_list.cc
  ./multithread_search/gbfs_independent.cc
  ./multithread_search/gbfs_shared_closed.cc
  ./multithread_search/gbfs_shared_open.cc
  ./multithread_search/kpgbfs.cc
  ./multithread_search/kpgbfs_dump.cc
  ./multithread_search/hkpgbfs.cc
  ./multithread_search/puhf.cc
  ./multithread_search/tkpgbfs.cc
  ./multithread_search/tkplg.cc
  ./multithread_search/kplg.cc
  ./multithread_search/kplg_dump.cc
  ./multithread_search/puhf2.cc
  #./multithread_search/gbfs_portfolio.cc
  #./multithread_search/multi_focus_gbfs.cc
  #./multithread_search/independent_context.cc
  #./multithread_search/mcs_independent.cc
  #./multithread_search/mcs_dump.cc
  #./multithread_search/greedy_pbnf.cc
)

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
