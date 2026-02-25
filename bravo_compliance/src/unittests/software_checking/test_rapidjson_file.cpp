#include <iostream>
#include <fstream>   // for std::ifstream
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

int main() {
    // Open a JSON file (make sure test.json exists in the same folder)
    std::ifstream ifs("/home/carlos/cp_unite_ws/src/bravo7_version_2/config/test.json");
    if (!ifs.is_open()) {
        std::cerr << "❌ Failed to open file: test.json\n";
        return 1;
    }

    // Wrap the file stream for RapidJSON
    rapidjson::IStreamWrapper isw(ifs);

    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError()) {
        std::cerr << "❌ Failed to parse JSON\n";
        return 1;
    }

    // Access members
    if (doc.HasMember("hello") && doc["hello"].IsString()) {
        std::cout << "hello = " << doc["hello"].GetString() << std::endl;
    }

    if (doc.HasMember("number") && doc["number"].IsInt()) {
        std::cout << "number = " << doc["number"].GetInt() << std::endl;
    }

    std::cout << "✅ RapidJSON file parsing works fine!" << std::endl;
    return 0;
}