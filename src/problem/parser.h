#ifndef PARSER_H_
#define PARSER_H_

#include <string>

#include "domain/domain.h"

namespace rwls {

void Parse(const std::string &filename, Domain *domain);

} // namespace rwls

#endif // PARSER_H_
