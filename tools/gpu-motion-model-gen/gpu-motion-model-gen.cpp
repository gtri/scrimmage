#include <unistd.h>

#include <cstring>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <sstream>
#include <fstream>
#include <map>
#include <regex>
#include <vector>

namespace fs = std::filesystem;

struct EnumDef {
  std::string name;
  std::vector<std::string> enumerations;

  const std::regex name_regex{R"_((enum\s+)([a-zA-Z]+\w*)(\s+\{))_"};
  const std::regex enumeration_regex{R"_(([a-zA-Z]+\w*(\s*=\s*[^\s,\}]+)?)(?=(,|\s*\})))_"};


  bool init(const std::string& raw) {
    // Only expect named enums
    std::smatch declaration_match;
    std::regex_search(raw, declaration_match, name_regex);
    if (!declaration_match.ready() || declaration_match.empty()) {
      return false;
    }
    name = declaration_match.str(2);

    auto enumerations_begin = std::sregex_iterator{raw.begin(), raw.end(), enumeration_regex};
    auto enumerations_end = std::sregex_iterator{};
    for (auto match = enumerations_begin; match != enumerations_end; ++match) {
      enumerations.push_back(match->str());
    }
    return true;
  }

  static std::vector<EnumDef> parse_enums(const std::string& raw) {
    const std::regex enum_regex{R"_(enum[\w\s]+\{[^\}]+\})_"};
    std::vector<EnumDef> enums;
    auto enums_begin = std::sregex_iterator{raw.begin(), raw.end(), enum_regex};
    auto enums_end = std::sregex_iterator{};

    for(auto match = enums_begin; match != enums_end; ++match) {
      EnumDef enum_def;
      if(enum_def.init(match->str())) {
        enums.push_back(std::move(enum_def));
      }
    }
    return enums;
  }

  std::string string() const {
    std::string out;
    out += "enum " + name + " {\n";
    for(const std::string& enumeration: enumerations) {
      out += "    " + enumeration + ",\n";
    }
    out += "};\n";
    return out;
  }
};

std::ostream& operator<<(std::ostream& lhs, const EnumDef& rhs) {
  return lhs << rhs.string();
};

fs::path motion_model_src;
fs::path kernel_target_dir;

const char *kernel_template = 
#include "generated/gpu_motion_model_template.cl"
;

void show_usage(const char* program_name) {
  fprintf(stderr, "Usage: %s PATH/TO/MOTION_MODEL.cpp PATH/TO/KERNEL/DIR\n\n", program_name);
}

void parse_args(int argc, char* argv[]) {
  if(argc < 3) {
    fprintf(stderr, "No Enough Paramters supplied.\n\n");
    show_usage(argv[0]);
    exit(EXIT_FAILURE); 
  }
  motion_model_src = argv[1];
  kernel_target_dir = argv[2];
  if (!fs::exists(motion_model_src)) {
    fprintf(stderr, "The source file %s does not exist!", motion_model_src.c_str());
    exit(-1);
  }

  if (!fs::exists(kernel_target_dir)) {
    fprintf(stderr, "The target directory %s does not exist!", kernel_target_dir.c_str());
    exit(-1);
  }
}

std::string test_enum {R"(enum test_name {
  FIELD_1 = 0xBEEF,
  FIELD_2 = 0b10101110,
  FIELD_3
})"};

std::string to_snake_case(const std::string& str) {
  std::regex word_sep{R"_(\B[A-Z])_"};
  return std::regex_replace(str, word_sep, "_$&");
}

int main(int argc, char* argv[]) {
  parse_args(argc, argv);
  // Creates a subdirectory with the same name as the src motion model
  fs::path kernel_subdir{kernel_target_dir / motion_model_src.stem() };
  fs::path kernel_file{kernel_subdir / motion_model_src.stem().replace_extension("cl") };

  const std::string kernel_name = motion_model_src.stem().string();
  const std::string kernel_name_snake = to_snake_case(kernel_name);
  std::string kernel_name_snake_upper = kernel_name_snake;
  std::string kernel_name_snake_lower = kernel_name_snake;
  std::transform(
      kernel_name_snake_upper.begin(),
      kernel_name_snake_upper.end(),
      kernel_name_snake_upper.begin(),
      [](auto c){ return std::toupper(c); }
      );

  std::transform(
    kernel_name_snake_lower.begin(),
    kernel_name_snake_lower.end(),
    kernel_name_snake_lower.begin(),
    [](auto c){ return std::tolower(c); }
      );

  if(!fs::exists(kernel_subdir)) {
    fs::create_directory(kernel_subdir);
  }
  std::ifstream src;
  std::ofstream out;
  std::stringstream buffer;
  src.open(motion_model_src);
  out.open(kernel_file);
  buffer << src.rdbuf(); 

  std::vector<EnumDef> enums = EnumDef::parse_enums(buffer.str());
  for(EnumDef& e : enums) {
    e.name = kernel_name + e.name;
    for(std::string& enumeration : e.enumerations) {
      enumeration = kernel_name_snake_upper + "_" + enumeration;
    }
  }

  std::string to_write{kernel_template};

  std::string enum_str = std::accumulate(
      enums.cbegin(), 
      enums.cend(),
      std::string{}, 
      [](const std::string lhs, const EnumDef rhs){
      return lhs + "\n" + rhs.string();
      });

  to_write = std::regex_replace(
      to_write,
      std::regex{R"_(\(>>>enums<<<\))_"},
      enum_str
      );

  to_write = std::regex_replace(
      to_write,
      std::regex{R"_(\(>>>KERNEL_NAME<<<\))_"},
      kernel_name
      );

  to_write = std::regex_replace(
      to_write,
      std::regex{R"_(\(>>>KERNEL_NAME_LOWER_SNAKE<<<\))_"},
      kernel_name_snake_lower
      );

  to_write = std::regex_replace(
      to_write,
      std::regex{R"_(\(>>>KERNEL_NAME_UPPER_SNAKE<<<\))_"},
      kernel_name_snake_upper
      );


  out << to_write;
  src.close();
  out.close();
}

