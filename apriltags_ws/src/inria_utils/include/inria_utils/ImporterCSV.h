#ifndef INRIA_UTILS_IMPORTERCSV_H
#define INRIA_UTILS_IMPORTERCSV_H

#include <string>
#include <fstream>
#include <vector>
#include <stdexcept>

namespace inria {

/**
 * Parse and explode given line into floating number split by delimiter.
 *
 * @param line String line to be split.
 * @param CSV delimiter character.
 * @param data Push extracted number into given container.
 */
inline void ParseLineFloat(
    const std::string& line, char delimiter, std::vector<double>& data)
{
    size_t tmpPosStart = 0;
    while (true) {
        if (tmpPosStart == std::string::npos || tmpPosStart >= line.length()) {
            break;
        }
        while (
            tmpPosStart < line.length() && 
            !std::isdigit(line[tmpPosStart]) && 
            line[tmpPosStart] != '+' &&
            line[tmpPosStart] != '-'
        ) {
            tmpPosStart++;
        }
        size_t tmpPosEnd = line.find(delimiter, tmpPosStart);
        if (tmpPosEnd == std::string::npos) {
            tmpPosEnd = line.length();
        }
        if (tmpPosEnd > tmpPosStart) {
            double val = std::stold(line.substr(tmpPosStart, tmpPosEnd-tmpPosStart));
            data.push_back(val);
        }
        tmpPosStart = tmpPosEnd + 1;
    }
}

/**
 * Parse and explode given line into string split by delimiter.
 *
 * @param line String line to be split.
 * @param CSV delimiter character.
 * @param data Push extracted string into given container.
 */
inline void ParseLineStr(
    const std::string& line, char delimiter, std::vector<std::string>& data)
{
    size_t tmpPosStart = 0;
    while (true) {
        if (tmpPosStart == std::string::npos || tmpPosStart >= line.length()) {
            break;
        }
        while (
            tmpPosStart < line.length() && 
            line[tmpPosStart] == delimiter
        ) {
            tmpPosStart++;
        }
        size_t tmpPosEnd = line.find(delimiter, tmpPosStart);
        if (tmpPosEnd == std::string::npos) {
            tmpPosEnd = line.length();
        }
        if (tmpPosEnd > tmpPosStart) {
            data.push_back(line.substr(tmpPosStart, tmpPosEnd-tmpPosStart));
        }
        tmpPosStart = tmpPosEnd + 1;
    }
}

/**
 * ImporterCSV
 *
 * Import CSV data into nested vector container.
 *
 * @param filename Path to the CSV file.
 * @param delimiter CSV delimiter character.
 * @param header If not NULL, last line starting with '#' is store into it.
 * @return nested vector with rows x columns.
 */
inline std::vector<std::vector<double>> ImporterCSV(
    const std::string& filepath, char delimiter, 
    std::vector<std::string>* header = nullptr)
{
    std::vector<std::vector<double>> data;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error(
            "inria::ImporterCSV: error opening file: " + filepath);
    }
    while (file.good()) {
        std::string line;
        std::getline(file, line);
        if (line.length() == 0) {
            continue;
        }
        if (header != nullptr && line[0] == '#') {
            ParseLineStr(line.substr(1, line.length()-1), delimiter, *header);
            continue;
        }
        data.push_back({});
        ParseLineFloat(line, delimiter, data.back());
    }
    file.close();

    return data;
}

}

#endif

