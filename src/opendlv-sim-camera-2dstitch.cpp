/*
 * Copyright (C) 2020 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <vector>
#include <iostream>
#include <string>
#include <unordered_map>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "json.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

struct ModelInfo {
  std::string name;
  std::string modelFilename;

  ModelInfo(std::string const &a_name, std::string const &a_modelFilename):
    name(a_name),
    modelFilename(a_modelFilename) {}
};

struct Model {
  glm::vec2 xRange;
  glm::vec2 yRange;
  glm::vec2 zRange;
  std::vector<std::pair<uint32_t, stbi_uc *>> samples;

  Model():
    xRange(),
    yRange(),
    zRange(),
    samples() {}

  Model(glm::vec2 a_xRange, glm::vec2 a_yRange, glm::vec2 a_zRange):
    xRange(a_xRange),
    yRange(a_yRange),
    zRange(a_zRange),
    samples() {}
};



std::map<std::string, Model> loadModels(std::vector<ModelInfo> modelInfo,
    std::string const &mapPath, bool verbose)
{
  std::map<std::string, Model> models;

  for (auto info : modelInfo) {
    if (verbose) {
      std::cout << "Loading model '" << info.name << "'" << std::endl;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, 
          info.modelFilename.c_str())) {
        throw std::runtime_error(warn + err);
    }

    float xMin{std::numeric_limits<float>::max()};
    float xMax{0.0f};
    float yMin{std::numeric_limits<float>::max()};
    float yMax{0.0f};
    float zMin{std::numeric_limits<float>::max()};
    float zMax{0.0f};

    for (auto const &shape : shapes) {
      for (auto const &index : shape.mesh.indices) {
        float x = attrib.vertices[3 * index.vertex_index + 0];
        float y = attrib.vertices[3 * index.vertex_index + 1];
        float z = attrib.vertices[3 * index.vertex_index + 2];

        xMin = std::min(xMin, x);
        yMin = std::min(yMin, y);
        zMin = std::min(zMin, z);

        xMax = std::max(xMax, x);
        yMax = std::max(yMax, y);
        zMax = std::max(zMax, z);
      }
    }

    models[info.name] = Model(glm::vec2(xMin, xMax), glm::vec2(yMin, yMax),
        glm::vec2(zMin, zMax));


    std::string path = mapPath + "/" + info.name + ".png";

    int32_t w;
    int32_t h;
    int32_t c;
    stbi_uc *tex = stbi_load(path.c_str(), &w, &h, &c, STBI_rgb_alpha);
    if (tex == nullptr) {
      std::cerr << "Could not load sample image: " << path << std::endl;
    }

    models[info.name].samples.push_back({w, tex});
  }



/*
  for (auto model : models) {
    if (!model.textureFilename.empty()) {
      int32_t w;
      int32_t h;
      int32_t c;
      stbi_uc *tex = stbi_load(model.textureFilename.c_str(), &w, &h, &c,
          STBI_rgb_alpha);

      glGenTextures(1, &handles[model.name].textureId);
      glBindTexture(GL_TEXTURE_2D, handles[model.name].textureId);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
          GL_UNSIGNED_BYTE, reinterpret_cast<void *>(tex));

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

      glBindTexture(GL_TEXTURE_2D, 0);
    
      stbi_image_free(tex);
    }
  }
  */

  return models;
}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{EXIT_SUCCESS};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("map-path") 
      || 0 == commandlineArguments.count("map-path") 
      || 0 == commandlineArguments.count("freq") 
      || 0 == commandlineArguments.count("width") 
      || 0 == commandlineArguments.count("height")
      || 0 == commandlineArguments.count("fovy")) {
    std::cerr << argv[0] << " simulates a camera sensor." << std::endl;
    std::cerr << "Usage:   " << argv[0] << std::endl
      << "  --cid=<OD4 session> " << std::endl
      << "  --map-path=<Folder where the map is stored> " << std::endl
      << "  --freq=<Frequency of camera> " << std::endl
      << "  --width=<Width of the output image> " << std::endl
      << "  --height=<Height of the output image> " << std::endl
      << "  --fovy=<Camera vertical field of view> " << std::endl
      << "  [--timemod=<Time scale modifier for simulation speed, "
      << "default: 1.0>] "
      << "  [--frame-id=<The frame to use for the true position, default: 0>] " 
      << std::endl
      << "  [--x=<Mount X position (forward), default: 0.0>] " << std::endl
      << "  [--y=<Mount Y position (left), default: 0.0>] " << std::endl
      << "  [--z=<Mount Z position (up), default: 0.0>] " << std::endl
      << "  [--yaw=<Mount yaw angle, default: 0.0>] " << std::endl
      << "  [--pitch=<Mount pitch angle, default: 0.0>] " << std::endl
      << "  [--roll=<Mount roll angle, default: 0.0>] " << std::endl
      << "  [--name.i420=<Shared memory for I420 data, default: video0.i420>] " 
      << std::endl
      << "  [--name.argb=<Shared memory for ARGB data, default: video0.argb>] "
      << std::endl
      << "  [--verbose]" << std::endl << std::endl
      << "Example: " << argv[0] << " --cid=111 --frame-id=0 "
      << "--map-path=../resource/example_map --x=1.3 --z=0.5 "
      << "--width=1280 --height=720 --fovy=48.8 --freq=5 --verbose" 
      << std::endl;
    retCode = 1;
  } else {
    std::string const mapPath{commandlineArguments["map-path"]};
    std::string const nameI420{(commandlineArguments["name.i420"].size() != 0) 
      ? commandlineArguments["name.i420"] : "video0.i420"};
    std::string const nameArgb{(commandlineArguments["name.argb"].size() != 0) 
      ? commandlineArguments["name.argb"] : "video0.argb"};
    uint32_t const freq = std::stoi(commandlineArguments["freq"]);
    float const timemod = (commandlineArguments["timemod"].size() != 0) 
      ? std::stof(commandlineArguments["timemod"]) : 1.0f;
    uint32_t const width = std::stoi(commandlineArguments["width"]);
    uint32_t const height = std::stoi(commandlineArguments["height"]);
    float const fovy = std::stof(commandlineArguments["fovy"]);
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    uint32_t const frameId = (commandlineArguments["frame-id"].size() != 0)
      ? std::stoi(commandlineArguments["frame-id"]) : 0;
    bool const verbose{commandlineArguments.count("verbose") != 0};

    float const aspect = static_cast<float>(width) / static_cast<float>(height);
    
    glm::vec3 mountPos((commandlineArguments["x"].size() != 0) 
        ? std::stod(commandlineArguments["x"]) : 0.0,
        (commandlineArguments["y"].size() != 0) 
        ? std::stod(commandlineArguments["y"]) : 0.0,
        (commandlineArguments["z"].size() != 0) 
        ? std::stod(commandlineArguments["z"]) : 0.0);
    glm::quat mountRot(glm::vec3(
        (commandlineArguments["roll"].size() != 0) 
        ? std::stod(commandlineArguments["roll"]) : 0.0,
        (commandlineArguments["pitch"].size() != 0) 
        ? std::stod(commandlineArguments["pitch"]) : 0.0,
        (commandlineArguments["yaw"].size() != 0) 
        ? std::stod(commandlineArguments["yaw"]) : 0.0));

    uint32_t const memSize = width * height * 4;
    cluon::SharedMemory sharedMemoryArgb(nameArgb, memSize);
    cluon::SharedMemory sharedMemoryI420(nameI420, memSize);
    if (verbose) {
      std::clog << "Created shared memory " << nameArgb << " (" << memSize 
        << " bytes) for an ARGB image (width = " << width << ", height = " 
        << height << ")." << std::endl;
      std::clog << "Created shared memory " << nameI420 << " (" << memSize 
        << " bytes) for an I420 image (width = " << width << ", height = " 
        << height << ")." << std::endl;
    }

    Display *display = XOpenDisplay(nullptr);
    if (!display) {
      std::cerr << "Could not open X display" << std::endl;
      return -1;
    }
    Visual* visual = DefaultVisual(display, 0);
    Window win = XCreateSimpleWindow(display, RootWindow(display, 0), 0, 0, 
        width, height, 1, 0, 0); 
    XImage* ximage{nullptr};
    sharedMemoryArgb.lock();
    {
      ximage = XCreateImage(display, visual, 24, ZPixmap, 0, 
          sharedMemoryArgb.data(), width, height, 32, 0);
    }
    sharedMemoryArgb.unlock();
    XMapWindow(display, win);

    nlohmann::json json;
    {
      std::ifstream i(mapPath + "/map.json");
      i >> json;
    }

    struct MeshInstance {
      std::string name;
      glm::vec3 position;
      float rotation;
      bool visible;

      MeshInstance():
        name(), position(), rotation(), visible() {}

      MeshInstance(std::string a_name, glm::vec3 a_position, float a_rotation,
          bool a_visible): 
        name(a_name), position(a_position), rotation(a_rotation), 
        visible(a_visible) {}
    };

    std::map<std::string, Model> models;
    std::vector<MeshInstance> meshInstances;
    std::map<uint32_t, MeshInstance> meshInstancesFrame;
    if (json.find("model") != json.end()) {
      std::vector<ModelInfo> modelInfo;
      for (auto const &j : json["model"]) {
        std::string name = j["name"];
        std::string file = j["file"];
        modelInfo.push_back({name, mapPath + "/" + file});
        if (j.find("instances") != j.end()) {
          for (auto const &i : j["instances"]) {
            float x = i[0];
            float y = i[1];
            float z = i[2];
            float a = i[3];
            meshInstances.push_back({name, glm::vec3(x, y, z), a, true});
          }
        }
        if (j.find("frames") != j.end()) {
          for (auto const &i : j["frames"]) {
            meshInstancesFrame[i] = {name, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f,
              false};
          }
        }
      }
      models = loadModels(modelInfo, mapPath, verbose);
    }
    /*
    if (json.find("overlay") != json.end()) {
      for (auto const &j : json["overlay"]) {
        std::string name = j["name"];
        float d0 = static_cast<float>(j["dimension"][0]) * width;
        float d1 = static_cast<float>(j["dimension"][1]) * height;
        std::string textureFile = j["textureFile"];

        for (auto const &i : j["instances"]) {
          float x = static_cast<float>(i[0]) * width + 0.5f * d0;
          float y = static_cast<float>(i[1]) * height + 0.5f * d1;
          meshInstances.push_back({name, glm::vec3(x, y, 0.0), 0.0, true});
        }
      }
    }
    */
    
    std::mutex meshInstancesFrameMutex;

    bool hasFrame{false};
    glm::mat4 view(glm::mat4(1.0f));
    std::mutex viewMutex;
    auto onFrame{[&frameId, &mountPos, &mountRot, &view, &viewMutex, &hasFrame,
    &meshInstancesFrame, &meshInstancesFrameMutex](
        cluon::data::Envelope &&envelope)
      {
        std::lock_guard<std::mutex> lock(meshInstancesFrameMutex);
        double hpi = glm::pi<double>() / 2.0;
        auto frame = cluon::extractMessage<opendlv::sim::Frame>(
            std::move(envelope));
        glm::vec3 framePos(frame.x(), frame.y(), frame.z());
        double horizontalAngle = hpi - frame.yaw();

        uint32_t const senderStamp = envelope.senderStamp();
        if (frameId == senderStamp) {
          glm::vec3 position = framePos + mountPos;

          double verticalAngle = frame.pitch() + mountRot.y; 

          glm::vec3 direction(
              std::cos(verticalAngle) * std::sin(horizontalAngle), 
              std::cos(verticalAngle) * std::cos(horizontalAngle),
              std::sin(verticalAngle));

          glm::vec3 right = glm::vec3(std::sin(horizontalAngle + hpi),
              std::cos(horizontalAngle + hpi), 0);
          glm::vec3 up = glm::cross(right, direction);

          glm::vec3 positionFlip(position.x, position.y, position.z);
          view = glm::lookAt(positionFlip, positionFlip + direction, up);

          hasFrame = true;
        }

        if (meshInstancesFrame.find(senderStamp) != meshInstancesFrame.end()) {
          meshInstancesFrame[senderStamp].visible = true;
          meshInstancesFrame[senderStamp].position = framePos;
          meshInstancesFrame[senderStamp].rotation = 
            static_cast<float>(horizontalAngle);
        }
      }};
    
    glm::mat4 proj = glm::perspective(glm::radians(fovy), aspect, 0.1f, 100.0f);

    std::vector<uint8_t> buf(memSize);
    auto atFrequency{[&width, &height, &memSize, &buf, &sharedMemoryArgb,
      &sharedMemoryI420, &proj, &view, &hasFrame, &meshInstances,
      &meshInstancesFrame, &meshInstancesFrameMutex, &display, &win, &ximage,
      &verbose]() -> bool
      {
        cluon::data::TimeStamp sampleTimeStamp = cluon::time::now();

        if (hasFrame) {
          std::lock_guard<std::mutex> lock(meshInstancesFrameMutex);
          std::vector<MeshInstance> mis = meshInstances;
              
          for (auto const &mi : meshInstancesFrame) {
            if (mi.second.visible) {
              mis.push_back(mi.second);
            }
          }

          for (auto const &mi : mis) {
            glm::mat4 pv = proj * view;
            glm::vec4 p = pv * glm::vec4(mi.position, 1.0);
            uint32_t x = static_cast<uint32_t>(
                ((p.x / p.z + 1.0) / 2.0) * width + 0.5);
            uint32_t y = static_cast<uint32_t>(
                ((1.0 - p.y / p.z) / 2.0) * height + 0.5);

            if (mi.name == "cone_yellow") {
              std::cout << "x=" << x << " y=" << y << std::endl;
            }
          }
        }

        /*
        sharedMemoryArgb.lock();
        sharedMemoryArgb.setTimeStamp(sampleTimeStamp);
        {
          memcpy(sharedMemoryArgb.data(), buf.data(), memSize);
        }
        sharedMemoryArgb.unlock();
        sharedMemoryArgb.notifyAll();
        */

        /*
        sharedMemoryI420.lock();
        sharedMemoryI420.setTimeStamp(sampleTimeStamp);
        {
          memcpy(sharedMemoryI420.data(), buf.data(), memSize);
        }
        sharedMemoryI420.unlock();
        sharedMemoryI420.notifyAll();
        */

        if (verbose) {
          XPutImage(display, win, DefaultGC(display, 0), ximage, 0, 0, 0, 0, 
              width, height);
        }
        return true;
      }};


    cluon::OD4Session od4{cid};
    od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);
    od4.timeTrigger(timemod * freq, atFrequency);
        
    XDestroyWindow(display, win);
    XCloseDisplay(display);
    XFree(visual);

    // TODO: Do stbi_image_free(tex) on all images in the models....


  }

  return retCode;
}
