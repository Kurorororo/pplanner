SRC_DIR=./src
BIN_DIR=./bin
TEST_DIR=./tests

INCS = -I$(SRC_DIR) -I/usr/local/include/
LIBS = -L/usr/local/lib/ -lboost_program_options
CXX = g++
MPIXX = mpic++
RELEASE_FLAG = -Wall -std=c++11 -O3 -DNDEBUG
TEST_FLAG = -Wall -std=c++11 -Igoogletest/googletest/include \
						-Lgoogletest/googletest -lgtest -lgtest_main -lpthread

planner: \
	$(BIN_DIR)/planner.o \
	$(BIN_DIR)/successor_generator.o \
	$(BIN_DIR)/postprocess/action_elimination.o \
	$(BIN_DIR)/utils/file_utils.o \
	$(BIN_DIR)/libsas_plus.a \
	$(BIN_DIR)/libsearch_graph.a \
	$(BIN_DIR)/libevaluators.a \
	$(BIN_DIR)/libopen_lists.a \
	$(BIN_DIR)/libsearch.a
	$(CXX) $(INCS) $(RELEASE_FLAG) -o $(BIN_DIR)/planner \
		$(BIN_DIR)/planner.o \
		$(BIN_DIR)/successor_generator.o \
		$(BIN_DIR)/postprocess/action_elimination.o \
		$(BIN_DIR)/utils/file_utils.o \
		$(BIN_DIR)/libsearch.a \
		$(BIN_DIR)/libopen_lists.a \
		$(BIN_DIR)/libevaluators.a \
		$(BIN_DIR)/libsearch_graph.a \
		$(BIN_DIR)/libsas_plus.a \
		$(LIBS)

sas_relaxed_graphplan: \
	$(TEST_DIR)/heuristics/sas_relaxed_graphplan.cc \
	$(BIN_DIR)/heuristics/rpg.o \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/utils/file_utils.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(RELEASE_FLAG) -o $(BIN_DIR)/sas_relaxed_graphplan \
		$(TEST_DIR)/heuristics/sas_relaxed_graphplan.cc \
		$(BIN_DIR)/heuristics/rpg.o \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/utils/file_utils.o \
		$(BIN_DIR)/libsas_plus.a \
		$(LIBS)

sas_parser: \
	$(TEST_DIR)/sas_parser.cc \
	$(BIN_DIR)/utils/file_utils.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_parser \
		$(TEST_DIR)/sas_parser.cc \
		$(BIN_DIR)/utils/file_utils.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/%.o: $(SRC_DIR)/%.cc
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(INCS) $(RELEASE_FLAG) -o $@ -c $<

$(BIN_DIR)/libsas_plus.a: \
	$(BIN_DIR)/sas_plus.o \
	$(BIN_DIR)/sas_plus/partial_state.o \
	$(BIN_DIR)/sas_plus/partial_state_vector.o \
	$(BIN_DIR)/sas_plus/effect_vector.o \
	$(BIN_DIR)/sas_plus/facts.o \
	$(BIN_DIR)/sas_plus/mutex_groups.o \
	$(BIN_DIR)/sas_plus/parse_utils.o
	ar rcs $(BIN_DIR)/libsas_plus.a \
		$(BIN_DIR)/sas_plus.o \
		$(BIN_DIR)/sas_plus/partial_state.o \
		$(BIN_DIR)/sas_plus/partial_state_vector.o \
		$(BIN_DIR)/sas_plus/effect_vector.o \
		$(BIN_DIR)/sas_plus/facts.o \
		$(BIN_DIR)/sas_plus/mutex_groups.o \
		$(BIN_DIR)/sas_plus/parse_utils.o

$(BIN_DIR)/libsearch_graph.a: \
	$(BIN_DIR)/search_graph.o \
	$(BIN_DIR)/search_graph/state_vector.o \
	$(BIN_DIR)/search_graph/state_packer.o
	ar rcs $(BIN_DIR)/libsearch_graph.a \
		$(BIN_DIR)/search_graph.o \
		$(BIN_DIR)/search_graph/state_vector.o \
		$(BIN_DIR)/search_graph/state_packer.o

$(BIN_DIR)/libevaluators.a: \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/evaluator_factory.o \
	$(BIN_DIR)/heuristics/blind.o \
	$(BIN_DIR)/heuristics/new_operator.o \
	$(BIN_DIR)/heuristics/rpg.o \
	$(BIN_DIR)/heuristics/rpg_table.o \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/heuristics/width.o
	ar rcs $(BIN_DIR)/libevaluators.a \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/evaluator_factory.o \
		$(BIN_DIR)/heuristics/blind.o \
		$(BIN_DIR)/heuristics/new_operator.o \
		$(BIN_DIR)/heuristics/rpg.o \
		$(BIN_DIR)/heuristics/rpg_table.o \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/heuristics/width.o

$(BIN_DIR)/libopen_lists.a: \
	$(BIN_DIR)/open_list.o \
	$(BIN_DIR)/open_list_factory.o \
	$(BIN_DIR)/open_lists/single_open_list.o \
	$(BIN_DIR)/open_lists/preferred_open_list.o \
	$(BIN_DIR)/open_lists/open_list_impl_factory.o \
	$(BIN_DIR)/open_lists/open_list_impl.o
	ar rcs $(BIN_DIR)/libopen_lists.a \
		$(BIN_DIR)/open_list.o \
		$(BIN_DIR)/open_list_factory.o \
		$(BIN_DIR)/open_lists/single_open_list.o \
		$(BIN_DIR)/open_lists/preferred_open_list.o \
		$(BIN_DIR)/open_lists/open_list_impl_factory.o \
		$(BIN_DIR)/open_lists/open_list_impl.o

$(BIN_DIR)/libsearch.a: \
	$(BIN_DIR)/search.o \
	$(BIN_DIR)/search_factory.o \
	$(BIN_DIR)/search/gbfs.o \
	$(BIN_DIR)/search/mrw13.o
	ar rcs $(BIN_DIR)/libsearch.a \
		$(BIN_DIR)/search.o \
		$(BIN_DIR)/search_factory.o \
		$(BIN_DIR)/search/gbfs.o \
		$(BIN_DIR)/search/mrw13.o

$(BIN_DIR)/sas_plus/test_partial_state: \
	$(TEST_DIR)/sas_plus/test_partial_state.cc \
	$(BIN_DIR)/sas_plus/partial_state.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_plus/test_partial_state \
		$(TEST_DIR)/sas_plus/test_partial_state.cc \
		$(BIN_DIR)/sas_plus/partial_state.o

$(BIN_DIR)/sas_plus/test_partial_state_vector: \
	$(TEST_DIR)/sas_plus/test_partial_state_vector.cc \
	$(BIN_DIR)/sas_plus/partial_state_vector.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_plus/test_partial_state_vector \
		$(TEST_DIR)/sas_plus/test_partial_state_vector.cc \
		$(BIN_DIR)/sas_plus/partial_state_vector.o

$(BIN_DIR)/sas_plus/test_effect_vector: \
	$(TEST_DIR)/sas_plus/test_effect_vector.cc \
	$(BIN_DIR)/sas_plus/effect_vector.o \
	$(BIN_DIR)/sas_plus/partial_state_vector.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_plus/test_effect_vector \
		$(TEST_DIR)/sas_plus/test_effect_vector.cc \
		$(BIN_DIR)/sas_plus/effect_vector.o \
		$(BIN_DIR)/sas_plus/partial_state_vector.o

$(BIN_DIR)/sas_plus/test_facts: \
	$(TEST_DIR)/sas_plus/test_facts.cc \
	$(BIN_DIR)/sas_plus/facts.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_plus/test_facts \
		$(TEST_DIR)/sas_plus/test_facts.cc \
		$(BIN_DIR)/sas_plus/facts.o

$(BIN_DIR)/sas_plus/test_mutex_groups: \
	$(TEST_DIR)/sas_plus/test_mutex_groups.cc \
	$(BIN_DIR)/sas_plus/mutex_groups.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_plus/test_mutex_groups \
		$(TEST_DIR)/sas_plus/test_mutex_groups.cc \
		$(BIN_DIR)/sas_plus/mutex_groups.o

$(BIN_DIR)/test_sas_plus: \
	$(TEST_DIR)/test_sas_plus.cc \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/test_sas_plus \
		$(TEST_DIR)/test_sas_plus.cc \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/test_successor_generator: \
	$(TEST_DIR)/test_successor_generator.cc \
	$(BIN_DIR)/successor_generator.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/test_successor_generator \
		$(TEST_DIR)/test_successor_generator.cc \
		$(BIN_DIR)/successor_generator.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/hash/test_zobrist_hash: \
	$(TEST_DIR)/hash/test_zobrist_hash.cc \
	$(BIN_DIR)/hash/zobrist_hash.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/hash/test_zobrist_hash \
		$(TEST_DIR)/hash/test_zobrist_hash.cc \
		$(BIN_DIR)/hash/zobrist_hash.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/hash/test_array_hash: \
	$(TEST_DIR)/hash/test_array_hash.cc \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/hash/test_array_hash \
		$(TEST_DIR)/hash/test_array_hash.cc \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/search_graph/test_state_packer: \
	$(TEST_DIR)/search_graph/test_state_packer.cc \
	$(BIN_DIR)/search_graph/state_packer.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/search_graph/test_state_packer \
		$(TEST_DIR)/search_graph/test_state_packer.cc \
		$(BIN_DIR)/search_graph/state_packer.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/search_graph/test_state_vector: \
	$(TEST_DIR)/search_graph/test_state_vector.cc \
	$(BIN_DIR)/search_graph/state_vector.o \
	$(BIN_DIR)/search_graph/state_packer.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/search_graph/test_state_vector \
		$(TEST_DIR)/search_graph/test_state_vector.cc \
		$(BIN_DIR)/search_graph/state_vector.o \
		$(BIN_DIR)/search_graph/state_packer.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/test_search_graph: \
	$(TEST_DIR)/test_search_graph.cc \
	$(BIN_DIR)/libsas_plus.a \
	$(BIN_DIR)/libsearch_graph.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/test_search_graph \
		$(TEST_DIR)/test_search_graph.cc \
		$(BIN_DIR)/libsas_plus.a \
		$(BIN_DIR)/libsearch_graph.a

$(BIN_DIR)/heuristics/test_blind: \
	$(TEST_DIR)/heuristics/test_blind.cc \
	$(BIN_DIR)/heuristics/blind.o \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/heuristics/test_blind \
		$(TEST_DIR)/heuristics/test_blind.cc \
		$(BIN_DIR)/heuristics/blind.o \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/heuristics/test_relaxed_sas_plus: \
	$(TEST_DIR)/heuristics/test_relaxed_sas_plus.cc \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/heuristics/test_relaxed_sas_plus \
		$(TEST_DIR)/heuristics/test_relaxed_sas_plus.cc \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/heuristics/test_rpg_table: \
	$(TEST_DIR)/heuristics/test_rpg_table.cc \
	$(BIN_DIR)/heuristics/rpg_table.o \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/heuristics/test_rpg_table \
		$(TEST_DIR)/heuristics/test_rpg_table.cc \
		$(BIN_DIR)/heuristics/rpg_table.o \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/heuristics/test_additive: \
	$(TEST_DIR)/heuristics/test_additive.cc \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/heuristics/rpg_table.o \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/heuristics/test_additive \
		$(TEST_DIR)/heuristics/test_additive.cc \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/heuristics/rpg_table.o \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/heuristics/test_ff_add: \
	$(TEST_DIR)/heuristics/test_ff_add.cc \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/heuristics/rpg_table.o \
	$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/heuristics/test_ff_add \
		$(TEST_DIR)/heuristics/test_ff_add.cc \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/heuristics/rpg_table.o \
		$(BIN_DIR)/heuristics/relaxed_sas_plus.o \
		$(BIN_DIR)/libsas_plus.a

$(BIN_DIR)/open_lists/test_fifo_open_list_impl: \
	$(TEST_DIR)/open_lists/test_fifo_open_list_impl.cc \
	$(BIN_DIR)/open_lists/open_list_impl.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/open_lists/test_fifo_open_lists_impl \
		$(TEST_DIR)/open_lists/test_fifo_open_list_impl.cc \
		$(BIN_DIR)/open_lists/open_list_impl.o

$(BIN_DIR)/open_lists/test_open_list_impl_factory: \
	$(TEST_DIR)/open_lists/test_open_list_impl_factory.cc \
	$(BIN_DIR)/open_lists/open_list_impl_factory.o
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/open_lists/test_open_list_impl_factory \
		$(TEST_DIR)/open_lists/test_open_list_impl_factory.cc \
		$(BIN_DIR)/open_lists/open_list_impl_factory.o

$(BIN_DIR)/open_lists/test_single_open_list: \
	$(TEST_DIR)/open_lists/test_single_open_list.cc \
	$(BIN_DIR)/open_lists/single_open_list.o \
	$(BIN_DIR)/open_lists/open_list_impl_factory.o \
	$(BIN_DIR)/open_lists/open_list_impl.o \
	$(BIN_DIR)/heuristics/blind.o \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/open_list.o \
	$(BIN_DIR)/libsas_plus.a \
	$(BIN_DIR)/libsearch_graph.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/open_lists/test_single_open_list \
		$(TEST_DIR)/open_lists/test_single_open_list.cc \
		$(BIN_DIR)/open_lists/single_open_list.o \
		$(BIN_DIR)/open_lists/open_list_impl_factory.o \
		$(BIN_DIR)/open_lists/open_list_impl.o \
		$(BIN_DIR)/heuristics/blind.o \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/open_list.o \
		$(BIN_DIR)/libsas_plus.a \
		$(BIN_DIR)/libsearch_graph.a

$(BIN_DIR)/open_lists/test_preferred_open_list: \
	$(TEST_DIR)/open_lists/test_preferred_open_list.cc \
	$(BIN_DIR)/open_lists/preferred_open_list.o \
	$(BIN_DIR)/open_lists/open_list_impl_factory.o \
	$(BIN_DIR)/open_lists/open_list_impl.o \
	$(BIN_DIR)/heuristics/blind.o \
	$(BIN_DIR)/evaluator.o \
	$(BIN_DIR)/open_list.o \
	$(BIN_DIR)/libsas_plus.a \
	$(BIN_DIR)/libsearch_graph.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/open_lists/test_preferred_open_list \
		$(TEST_DIR)/open_lists/test_preferred_open_list.cc \
		$(BIN_DIR)/open_lists/preferred_open_list.o \
		$(BIN_DIR)/open_lists/open_list_impl_factory.o \
		$(BIN_DIR)/open_lists/open_list_impl.o \
		$(BIN_DIR)/heuristics/blind.o \
		$(BIN_DIR)/evaluator.o \
		$(BIN_DIR)/open_list.o \
		$(BIN_DIR)/libsas_plus.a \
		$(BIN_DIR)/libsearch_graph.a

clean:
	rm -rf $(BIN_DIR)/*

