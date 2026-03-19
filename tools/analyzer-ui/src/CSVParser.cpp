#include "CSVParser.h"
#include <fstream>
#include <sstream>

bool CSVParser::parse(const std::string &filename, CSVData &outData) {
  outData.headers.clear();
  outData.columns.clear();
  outData.timestamps.clear();

  std::ifstream file(filename);
  if (!file.is_open())
    return false;

  std::string line;
  if (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      if (!token.empty() && token.back() == '\r') {
        token.pop_back();
      }
      outData.headers.push_back(token);
      if (token != "timestamp_us") {
        outData.columns[token] = std::vector<double>();
      }
    }
  }

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    int colIdx = 0;
    double timestamp = 0.0;

    while (std::getline(ss, token, ',')) {
      if (colIdx < outData.headers.size()) {
        const auto &header = outData.headers[colIdx];
        double val = 0.0;
        try {
          if (!token.empty())
            val = std::stod(token);
        } catch (...) {
        }

        if (header == "timestamp_us") {
          timestamp = val;
        } else {
          outData.columns[header].push_back(val);
        }
      }
      colIdx++;
    }

    if (colIdx > 0) {
      outData.timestamps.push_back(timestamp);
      for (size_t i = colIdx; i < outData.headers.size(); i++) {
        const auto &header = outData.headers[i];
        if (header != "timestamp_us") {
          outData.columns[header].push_back(0.0);
        }
      }
    }
  }

  return true;
}
