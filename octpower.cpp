// Copyright 2019-2020 Richard Lincoln. All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include <oct.h>
#include <octave.h>
#include <parse.h>
#include <interpreter.h>
#include <builtin-defun-decls.h>

#include <emscripten.h>
#include <emscripten/bind.h>

#include <nlohmann/json.hpp>

using namespace emscripten;
using json = nlohmann::json;

int matrix_to_json(const octave_value& val, json& j) {
  if (!val.is_matrix_type()) {
    return 1;
  }
  if (!j.is_array()) {
    return 1;
  }

  Matrix matrix = val.matrix_value();

  octave_idx_type rows = matrix.rows();
  octave_idx_type cols = matrix.cols();

  for (octave_idx_type i = 0; i < rows; i++) {
    json row_array = json::array();

    for (octave_idx_type j = 0; j < cols; j++) {
      octave_value val = matrix(i, j);
      double sv = val.scalar_value();
      if (sv == std::numeric_limits<double>::infinity()) {
        sv = INT_MAX;
      } else if (sv == -std::numeric_limits<double>::infinity()) {
        sv = INT_MIN;
      }
      row_array.push_back(sv);
    }

    j.push_back(row_array);
  }
  return 0;
}

int json_to_matrix(const json& j, Matrix& matrix) {
  if (!j.is_array()) {
    return 1;
  }

  // Set the matrix dimensions.
  octave_idx_type max_cols;
  for (auto& row : j) {
    if (row.size() > max_cols) {
      max_cols = row.size();
    }
  }
  matrix.resize(j.size(), max_cols);

  for (int r = 0; r < j.size(); r++) {
    auto& row = j.at(r);
    for (int c = 0; c < row.size(); c++) {
      matrix(r, c) = row.at(c).get<double>();
    }
  }
  return 0;
}

int mpc_to_json(const octave_value& val, json& j) {
  if (!val.isstruct()) {
    return 1;
  }
  octave_scalar_map sm = val.scalar_map_value();

  if (sm.contains("baseMVA")) {
    octave_value val = sm.getfield("baseMVA");
    j["baseMVA"] = val.scalar_value();
  }

  if (sm.contains("bus")) {
    octave_value val = sm.getfield("bus");
    json m = json::array();
    if (matrix_to_json(val, m)) {
      return 2;
    }
    j["bus"] = m;
  }
  if (sm.contains("gen")) {
    octave_value val = sm.getfield("gen");
    json m = json::array();
    if (matrix_to_json(val, m)) {
      return 3;
    }
    j["gen"] = m;
  }
  if (sm.contains("branch")) {
    octave_value val = sm.getfield("branch");
    json m = json::array();
    if (matrix_to_json(val, m)) {
      return 3;
    }
    j["branch"] = m;
  }
  if (sm.contains("gencost")) {
    octave_value val = sm.getfield("gencost");
    json m = json::array();
    if (matrix_to_json(val, m)) {
      return 3;
    }
    j["gencost"] = m;
  }
  if (sm.contains("dcline")) {
    octave_value val = sm.getfield("dcline");
    json m = json::array();
    if (matrix_to_json(val, m)) {
      return 4;
    }
    j["dcline"] = m;
  }

  return 0;
}

int json_to_mpc(const json& j, octave_scalar_map& mpc) {
  if (j.find("baseMVA") != j.end()) {
    octave_value val(j["baseMVA"].get<double>());
    mpc.setfield("baseMVA", val);
  }

  if (j.find("bus") != j.end()) {
    Matrix matrix;
    if (json_to_matrix(j["bus"], matrix)) {
      return 2;
    }
    mpc.setfield("bus", matrix);
  }
  if (j.find("gen") != j.end()) {
    Matrix matrix;
    if (json_to_matrix(j["gen"], matrix)) {
      return 3;
    }
    mpc.setfield("gen", matrix);
  }
  if (j.find("branch") != j.end()) {
    Matrix matrix;
    if (json_to_matrix(j["branch"], matrix)) {
      return 4;
    }
    mpc.setfield("branch", matrix);
  }
  if (j.find("gencost") != j.end()) {
    Matrix matrix;
    if (json_to_matrix(j["gencost"], matrix)) {
      return 5;
    }
    mpc.setfield("gencost", matrix);
  }
  if (j.find("dcline") != j.end()) {
    Matrix matrix;
    if (json_to_matrix(j["dcline"], matrix)) {
      return 6;
    }
    mpc.setfield("dcline", matrix);
  }

  return 0;
}

std::string json_error(std::string msg) {
  json j;
  j["err"] = msg;
  return j.dump();
}

std::string EMSCRIPTEN_KEEPALIVE load_case(std::string casename) {
  try {
    octave_value_list in;
    in(0) = casename;
    octave_value_list out = octave::feval("loadcase", in, 1);

    if (out.length() == 0) {
      return json_error("error loading case: " + casename);
    }

    octave_value val = out(0);
    json mpc = json::object();

    if (mpc_to_json(val, mpc)) {
      return json_error("error converting mpc to json");
    }
    return mpc.dump();
  } catch (const octave::exit_exception& ex) {
    return json_error("interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

std::string EMSCRIPTEN_KEEPALIVE save_case(std::string mpc_data, std::string casename) {
  try {
    // Parse the input JSON and convert to a MPC.
    json mpc_json = json::parse(mpc_data);
    octave_scalar_map mpc;
    if (json_to_mpc(mpc_json, mpc)) {
      return json_error("error converting json to mpc");
    }

    std::string filename("/tmp/" + casename + ".m");

    octave_value_list in;
    in(0) = filename;
    in(1) = mpc;
    octave_value_list out = octave::feval("savecase", in, 1);

    if (out.length() == 0) {
      return json_error("error saving case: " + casename);
    }

    std::ifstream infile(filename);
    std::stringstream buffer;
    buffer << infile.rdbuf();

    json retval_json;
    retval_json["mpc"] = buffer.str();
    return retval_json.dump();
  } catch (const octave::exit_exception& ex) {
    return json_error("interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

std::string EMSCRIPTEN_KEEPALIVE read_raw(std::string raw_data, int rev) {
  try {
    std::string rawfile = std::tmpnam(nullptr);

    std::ofstream outfile(rawfile);
    outfile << raw_data;
    outfile.close();

    octave_value_list in;
    in(0) = rawfile;
    in(1) = 0; // verbose
    in(2) = rev;
    octave_value_list out = octave::feval("psse2mpc", in, 1);

    if (out.length() == 0) {
      return json_error("error converting raw to mpc: " + rawfile);
    }

    octave_value val = out(0);
    json mpc = json::object();

    if (mpc_to_json(val, mpc)) {
      return json_error("error converting mpc to json");
    }
    return mpc.dump();
  } catch (const octave::exit_exception& ex) {
    return json_error("interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

std::string EMSCRIPTEN_KEEPALIVE save_raw(std::string mpc_data) {
  try {
    // Parse the input JSON and convert to a MPC.
    json mpc_json = json::parse(mpc_data);
    octave_scalar_map mpc;
    if (json_to_mpc(mpc_json, mpc)) {
      return json_error("error converting json to mpc");
    }

    std::string filename = std::tmpnam(nullptr);

    octave_value_list in;
    in(0) = filename;
    in(1) = mpc;
    octave_value_list out = octave::feval("save2psse", in, 1);

    if (out.length() == 0) {
      return json_error("error saving raw: " + filename);
    }

    std::ifstream infile(out(0).string_value());
    std::stringstream buffer;
    buffer << infile.rdbuf();

    json raw_json;
    raw_json["raw"] = buffer.str();
    return raw_json.dump();
  } catch (const octave::exit_exception& ex) {
    return json_error("interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

std::string runf(std::string name, std::string mpc_data, std::string mpopt_data) {
  // Parse the input JSON and convert to a MPC.
  json mpc_json = json::parse(mpc_data);
  octave_scalar_map mpc0;
  if (json_to_mpc(mpc_json, mpc0)) {
    return json_error("error converting json to mpc");
  }

  octave_value mpc = mpc0;
  if (mpc_json.find("dcline") != mpc_json.end() && mpc_json["dcline"].size() != 0) {
    octave_value_list dcline_args;
    dcline_args(0) = mpc0;
    dcline_args(1) = "on";
    octave_value_list dcline_argout = octave::feval("toggle_dcline", dcline_args, 1);
    mpc = dcline_argout(0);
  }

  // Set up a MATPOWER options struct.
  octave_value_list mpopt_args;
  mpopt_args.append("out.all");
  mpopt_args.append(0);

  json mpopt_json = json::parse(mpopt_data);
  if (mpopt_json.find("verbose") != mpopt_json.end()) {
    mpopt_args.append("verbose");
    mpopt_args.append(mpopt_json["verbose"].get<int>());
  }
  if (mpopt_json.find("model") != mpopt_json.end()) {
    mpopt_args.append("model");
    mpopt_args.append(mpopt_json["model"].get<std::string>());
  }
  if (mpopt_json.find("pf") != mpopt_json.end()) {
    json pf = mpopt_json["pf"].get<json>();

    if (pf.find("alg") != pf.end()) {
      mpopt_args.append("pf.alg");
      mpopt_args.append(pf["alg"].get<std::string>());
    }
    if (pf.find("current_balance") != pf.end()) {
      mpopt_args.append("pf.current_balance");
      mpopt_args.append(pf["current_balance"].get<int>());
    }
    if (pf.find("v_cartesian") != pf.end()) {
      mpopt_args.append("pf.v_cartesian");
      mpopt_args.append(pf["v_cartesian"].get<int>());
    }
    if (pf.find("tol") != pf.end()) {
      mpopt_args.append("pf.tol");
      mpopt_args.append(pf["tol"].get<double>());
    }
    if (pf.find("nr") != pf.end()) {
      json nr = pf["nr"].get<json>();
      if (nr.find("max_it") != nr.end()) {
        mpopt_args.append("pf.nr.max_it");
        mpopt_args.append(nr["max_it"].get<int>());
      }
    }
    if (pf.find("fd") != pf.end()) {
      json fd = pf["fd"].get<json>();
      if (fd.find("max_it") != fd.end()) {
        mpopt_args.append("pf.fd.max_it");
        mpopt_args.append(fd["max_it"].get<int>());
      }
    }
    if (pf.find("enforce_q_lims") != pf.end()) {
      mpopt_args.append("pf.enforce_q_lims");
      mpopt_args.append(pf["enforce_q_lims"].get<int>());
    }
  }
  if (mpopt_json.find("opf") != mpopt_json.end()) {
    json opf = mpopt_json["opf"].get<json>();

    if (opf.find("current_balance") != opf.end()) {
      mpopt_args.append("opf.current_balance");
      mpopt_args.append(opf["current_balance"].get<int>());
    }
    if (opf.find("v_cartesian") != opf.end()) {
      mpopt_args.append("opf.v_cartesian");
      mpopt_args.append(opf["v_cartesian"].get<int>());
    }
    if (opf.find("violation") != opf.end()) {
      mpopt_args.append("opf.violation");
      mpopt_args.append(opf["violation"].get<double>());
    }
    if (opf.find("flow_lim") != opf.end()) {
      mpopt_args.append("opf.flow_lim");
      mpopt_args.append(opf["flow_lim"].get<std::string>());
    }
    if (opf.find("ignore_angle_lim") != opf.end()) {
      mpopt_args.append("opf.ignore_angle_lim");
      mpopt_args.append(opf["ignore_angle_lim"].get<int>());
    }
    if (opf.find("start") != opf.end()) {
      mpopt_args.append("opf.start");
      mpopt_args.append(opf["start"].get<int>());
    }
  }
  octave_value_list mpopt_argout = octave::feval("mpoption", mpopt_args, 1);

  // Call runpf with the MPC and options.
  octave_value_list runpf_args;
  runpf_args(0) = mpc;
  runpf_args(1) = mpopt_argout(0);//.scalar_map_value();
  octave_value_list results_argout = octave::feval(name, runpf_args, 2);

  // Convert the results to a JSON string.
  json results_json = json::object();
  if (mpc_to_json(results_argout(0), results_json)) {
    return json_error("error converting " + name + " results to json");
  }

  // Check for successful completion.
  octave_value success = results_argout(1);
  if (!success.bool_value()) {
    results_json["success"] = false;
  }

  return results_json.dump();
}

std::string EMSCRIPTEN_KEEPALIVE runpf(std::string mpc_data, std::string mpopt_data) {
  try {
    return runf("runpf", mpc_data, mpopt_data);
  } catch (json::parse_error& e) {
    return json_error("runpf: error parsing mpc: " + std::string(e.what()));
  } catch (const octave::exit_exception& ex) {
    return json_error("runpf: interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("runpf: exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

std::string EMSCRIPTEN_KEEPALIVE runopf(std::string mpc_data, std::string mpopt_data) {
  try {
    return runf("runopf", mpc_data, mpopt_data);
  } catch (json::parse_error& e) {
    return json_error("runopf: error parsing mpc: " + std::string(e.what()));
  } catch (const octave::exit_exception& ex) {
    return json_error("runopf: interpreter exited with status = " + std::to_string(ex.exit_status()));
  } catch (const octave::execution_exception& ex) {
    std::cerr << ex.info() << std::endl;
    return json_error(last_error_message());
  } catch (const std::exception& ex) {
    return json_error("runopf: exception occurred: " + std::string(ex.what()));
  }
  return json_error("internal error");
}

int main(int argc, char **argv) {
  std::cout << "Starting GNU Octave interpreter..." << std::endl;

  static octave::interpreter interpreter;

  interpreter.initialize_load_path(true);
  interpreter.read_init_files(true);
  int status = interpreter.execute();
  if (status != 0) {
      std::cerr << "creating interpreter failed: " << status << std::endl;
      return status;
  }

  try {
    octave_value_list octave_paths;
    octave_paths(0) = octave_value("/usr/src/octave/m/help:"
        "/usr/src/octave/m/general:"
        "/usr/src/octave/m/set:"
        "/usr/src/octave/m/miscellaneous:"
        "/usr/src/octave/m/strings:"
        "/usr/src/octave/m/sparse:"
        "/usr/src/octave/m/path:"
        "/usr/src/octave/m/io:"
        "/usr/src/octave/m/polynomial:"
        "/usr/src/octave/m/pkg:"
        "/usr/src/octave/m/time", '\'');
    Faddpath(interpreter, octave_paths);

    octave_value_list matpower_paths;
    matpower_paths(0) = octave_value("/usr/src/matpower7.0", '\'');
    Faddpath(interpreter, matpower_paths);
  } catch (const octave::exit_exception& ex) {
    return ex.exit_status();
  } catch (const octave::execution_exception& ex) {
    std::cerr << "error adding paths: " << last_error_message() << std::endl;
    return 1;
  }

  try {
    octave_value_list in;
    in(0) = 1;
    in(1) = 1;
    in(2) = 0;
    octave_value_list out = octave::feval("install_matpower", in, 3);
  } catch (const octave::exit_exception& ex) {
    return ex.exit_status();
  } catch (const octave::execution_exception& ex) {
    std::cerr << "error installing matpower: " << last_error_message() << std::endl;
    return 2;
  }

  if (argc > 1) {
    if (std::strcmp(argv[1], "loadcase")) {
      std::string casename("case9");
      if (argc > 2) {
        casename = argv[2];
      }
      std::cout << load_case(casename) << std::endl;
    } else if (std::strcmp(argv[1], "runpf")) {
      std::string casedata;
      if (argc > 2) {
        casedata = argv[2];
      } else {
        std::getline(std::cin, casedata);
      }
      std::cout << runpf(casedata, "") << std::endl;
    } else if (std::strcmp(argv[1], "runopf")) {
      std::string casedata;
      if (argc > 2) {
        casedata = argv[2];
      } else {
        std::getline(std::cin, casedata);
      }
      std::cout << runopf(casedata, "") << std::endl;
    } else {
      std::cerr << "invalid command: " << argv[1] << std::endl;
      return 2;
    }
  }

  return 0;
}

EMSCRIPTEN_BINDINGS(my_module) {
  function("load_case", &load_case);
  function("save_case", &save_case);
  function("read_raw", &read_raw);
  function("save_raw", &save_raw);
  function("runpf", &runpf);
  function("runopf", &runopf);
  function("last_error_message", &last_error_message);
}

