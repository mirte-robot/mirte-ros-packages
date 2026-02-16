#pragma once

#include <optional>
#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
inline std::optional<int> try_parse_int(std::string str) {
  try {
    return boost::lexical_cast<int>(str);
  } catch (...) {
    return {};
  }
}
inline std::optional<double> try_parse_double(std::string str) {
  try {
    return boost::lexical_cast<double>(str);
  } catch (...) {
    return {};
  }
}
inline bool starts_with(std::string str, std::string prefix) {
  return str.rfind(prefix, 0) == 0;
}

#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

std::string exec(const std::string &cmd);

#include <filesystem>
#include <iostream>
#include <string>
