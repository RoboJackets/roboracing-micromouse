#pragma once
#include <map>
#include <string>
#include <vector>

struct CSVData {
  std::vector<std::string> headers;
  std::map<std::string, std::vector<double>> columns;
  std::vector<double> timestamps;
};

class CSVParser {
public:
  static bool parse(const std::string &filename, CSVData &outData);
};
