---
layout: note_page
title: C++ Logger 함수
tags: [C++]
category: "Programming"
---

C++에서 사용할 수 있는 Logger 클래스이다. 해당 클래스는 Logger의 기본적인 기능만 다루고 있으므로, 부가 기능이 필요한 경우 추가 개발해서 사용해서 보완하면 좋겠다.

# LOGGER

ROS에서 자동으로 log를 생성하지만, log를 깔끔하게 정리하기 어렵다. 따라서 C++ Logger 클래스를 만들어, log를 생성하는 기능을 추가하고자 한다.

해당 내용은 [DVELOPER 블로그의 'C++로 Log 남기는 방법 – Logger 만들기'](https://devbin.kr/2021/c%EB%A1%9C-log-%EB%82%A8%EA%B8%B0%EB%8A%94-%EB%B0%A9%EB%B2%95-logger-%EB%A7%8C%EB%93%A4%EA%B8%B0/) 포스트의 코드를 기반으로 작성하였다.

다음은 Logger 클래스의 헤더파일이다.

```cpp
#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <iomanip>
#include <iostream>
#include <fstream>
#include <cstdarg>
#include <ctime>
#include <sstream>
#include <cstring>
#include <cstdio>
#include <experimental/filesystem>

#define LOG_LEVEL_OFF 0
#define LOG_LEVEL_FATAL 10
#define LOG_LEVEL_ERROR 20
#define LOG_LEVEL_WARN 30
#define LOG_LEVEL_INFO 40
#define LOG_LEVEL_DEBUG 50
#define LOG_LEVEL_TRACE 60
#define LOG_LEVEL_ALL 100

#define __LOG_FILE__ "./logtest/log.txt"
#define fatal(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_FATAL, str, ##__VA_ARGS__)
#define error(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_ERROR, str, ##__VA_ARGS__)
#define warn(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_WARN, str, ##__VA_ARGS__)
#define info(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_INFO, str, ##__VA_ARGS__)
#define debug(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_DEBUG, str, ##__VA_ARGS__)
#define trace(str, ...) writeLog(__FUNCTION__, __LINE__, LOG_LEVEL_TRACE, str, ##__VA_ARGS__)

using namespace std;
namespace fs = std::experimental::filesystem;

class Logger
{
public:
  Logger();
  Logger(int level);
  Logger(int level, string location, string name);
  Logger(int level, string location, string name, string del_location);

  /**
   * @brief Logger 클래스 생성자
   *
   * @param level 로그를 기록할 수준으로, 해당 수준 이상의 로그만 기록됨
   * @param location 로그를 저장할 위치
   * @param name 로그 이름
   * @param del_location 로그를 삭제할 위치
   * @param del_days 로그를 삭제할 기준 날짜
   */
  Logger(int level, string location, string name, string del_location, int del_days);
  void writeLog(const char *funcName, int line, int level, const char *str, ...);
  void deleteLog(string base_path, int days = 30);

private:
  int logLevel; // 터미널에 출력할 로그 수준
  string getTimestamp();
  string log_folder; // 로그를 저장할 디렉토리 위치
  string log_file; // 로그를 저장할 파일 이름
};

#endif // __LOGGER_H__
```

다음은 소스 파일이다. 로그 파일의 폴더 구조는 특정 베이스 폴더 아래에 `2023-01-17`와 같은 폴더를 만들고, 그 아래에 `log_file`로 지정한 파일 이름으로 구성되어있다. 따라서 log 삭제 시, 베이스 폴더 아래의 날짜 폴더의 이름인 날짜를 현재 날짜와 비교하여 기준보다 오래된 폴더면 하위 폴더까지 삭제하도록 설정하였다.

```cpp
#include <logger.hpp>

Logger::Logger()
{
  this->logLevel = LOG_LEVEL_ERROR;
  log_folder = "";
  log_file = "log.txt";
}

Logger::Logger(int level)
{
  this->logLevel = level;
  log_folder = "";
  log_file = "log.txt";
}

Logger::Logger(int level, string location, string name)
{
  this->logLevel = level;
  log_folder = location;
  log_file = name;

  fs::create_directories(log_folder);
}

Logger::Logger(int level, string location, string name, string del_location)
{
  this->logLevel = level;
  log_folder = location;
  log_file = name;

  fs::create_directories(log_folder);

  deleteLog(del_location);
}

Logger::Logger(int level, string location, string name, string del_location, int del_days)
{
  this->logLevel = level;
  log_folder = location;
  log_file = name;

  fs::create_directories(log_folder);

  deleteLog(del_location, del_days);
}

string Logger::getTimestamp()
{
  string result;
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  ostringstream oss;

  oss.clear();
  oss << put_time(localtime(&t), "%Y-%m-%d-%H-%M-%S-");
  oss << std::setfill('0') << std::setw(3) << ms % 1000;

  result = result + oss.str();

  return result;
}

void Logger::writeLog(const char *funcName, int line, int lv, const char *str, ...)
{
  FILE *fp = NULL;
  string file = log_folder + log_file;
  fp = fopen(file.c_str(), "a");

  if(fp == NULL)
  {
    puts("fail to open file pointer");
    return;
  }

  char *result = NULL;
  char level[10];

  switch(lv)
  {
    case(LOG_LEVEL_FATAL): strcpy(level, "[FATAL]"); break;
    case(LOG_LEVEL_ERROR): strcpy(level, "[ERROR]"); break;
    case(LOG_LEVEL_WARN): strcpy(level, "[WARN] "); break;
    case(LOG_LEVEL_INFO): strcpy(level, "[INFO] "); break;
    case(LOG_LEVEL_DEBUG): strcpy(level, "[DEBUG]"); break;
    case(LOG_LEVEL_TRACE): strcpy(level, "[TRACE]"); break;
  }

  result = (char*)malloc(sizeof(char)*(21+strlen(funcName)+strlen(str)+30));
  sprintf(result, "%s %s [%s:%d] : %s\n", level, getTimestamp().c_str(), funcName, line, str);

  va_list args;

  va_start(args, str);
  vfprintf(fp, result, args);
  va_end(args);

  va_start(args, str);
  if(this->logLevel >= lv)
  {
    vprintf(result, args);
  }
  va_end(args);

  if(result != NULL)
  {
    free(result);
  }
  if(fp != NULL)
  {
    fclose(fp);
  }

  return;
}

void Logger::deleteLog(string base_path, int days)
{
  for( auto& p : std::experimental::filesystem::directory_iterator(base_path) )
  {
    string file = p.path().string().substr(p.path().string().length()-10);

    std::tm tm = {};
    std::stringstream ss(file);
    ss >> std::get_time(&tm, "%Y-%m-%d");
    auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));

    auto curr_time = chrono::system_clock::now();

    std::chrono::duration<double, kilo> sec = curr_time-tp;

    // 1일 기준 시간(초) = 86400
    if(sec.count() > 86400 * days * 0.001) {
      fs::remove_all(p);
    }
  }
  return;
}
```

위에서 구현한 Logger 클래스에 대한 사용 방법은 다음과 같다.

```cpp
#include <logger.hpp>

int main()
{
  // 베이스 폴더 및 날짜 폴더 자동 생성 함수 필요
  auto logger = std::make_shared<RCS::Logger>(LOG_LEVEL_WARN, "base/folder/location/2023-01-17/", "test.txt", "base/folder/location/", 10);
  logger->fatal("fatal %d", 123);
  logger->error("error");
  logger->warn("warn");
  logger->info("info");
  logger->debug("debug");
  logger->trace("trace");

  return 0;
}
```
