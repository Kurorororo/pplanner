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

$(BIN_DIR)/%.o: $(SRC_DIR)/%.cc
	@if [ ! -e `dirname $@` ]; then mkdir -p `dirname $@`; fi
	$(CXX) $(INCS) $(RELEASE_FLAG) -o $@ -c $<

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

$(BIN_DIR)/libsearch_graph.a: \
	$(BIN_DIR)/search_graph.o \
	$(BIN_DIR)/search_graph/state_vector.o \
	$(BIN_DIR)/search_graph/state_packer.o
	ar rcs $(BIN_DIR)/libsearch_graph.a \
		$(BIN_DIR)/search_graph.o \
		$(BIN_DIR)/search_graph/state_vector.o \
		$(BIN_DIR)/search_graph/state_packer.o \

$(BIN_DIR)/test_search_graph: \
	$(TEST_DIR)/test_search_graph.cc \
	$(BIN_DIR)/libsas_plus.a \
	$(BIN_DIR)/libsearch_graph.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/test_search_graph \
		$(TEST_DIR)/test_search_graph.cc \
		$(BIN_DIR)/libsas_plus.a \
		$(BIN_DIR)/libsearch_graph.a

sas_parser: \
	$(TEST_DIR)/sas_parser.cc \
	$(BIN_DIR)/utils/file_utils.o \
	$(BIN_DIR)/libsas_plus.a
	$(CXX) $(INCS) $(TEST_FLAG) -o $(BIN_DIR)/sas_parser \
		$(TEST_DIR)/sas_parser.cc \
		$(BIN_DIR)/utils/file_utils.o \
		$(BIN_DIR)/libsas_plus.a

clean:
	rm -rf $(BIN_DIR)/*

