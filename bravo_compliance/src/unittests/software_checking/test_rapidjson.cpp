#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

int main() {
    // JSON string to parse
    const char* json = R"({"hello": "world", "number": 42})";

    rapidjson::Document doc;
    doc.Parse(json);

    if (doc.HasParseError()) {
        std::cerr << "❌ Failed to parse JSON\n";
        return 1;
    }

    if (doc.HasMember("hello")) {
        std::cout << "hello = " << doc["hello"].GetString() << std::endl;
    }

    if (doc.HasMember("number")) {
        std::cout << "number = " << doc["number"].GetInt() << std::endl;
    }

    std::cout << "✅ RapidJSON works fine!" << std::endl;
    return 0;
}