---
layout: note_page
title: C++ REST SDK
tags: [REST API, Communication, C++]
category: "Dev"
---

## 1. SERVER

std::map 자료형의 dictionary 변수를 이용하여 client의 request 처리

#### main 함수

- http_listener 객체 생성(URI 지정)
- support() 함수로 request method 지정 및 handler 함수 지정
- listener loop 실행

```cpp
#include <cpprest/http_listener.h>
#include <cpprest/json.h>

using namespace web;
using namespace web::http;
using namespace web::http::experimental::listener;

#include <iostream>
#include <map>
#include <set>
#include <string>
using namespace std;

#define TRACE(msg)            cout << msg
#define TRACE_ACTION(a, k, v) cout << a << U(" (") << k << U(", ") << v << U(")\n")

map<utility::string_t, utility::string_t> dictionary;

void display_json(
   json::value const & jvalue,
   utility::string_t const & prefix)
{
   cout << prefix << jvalue.serialize() << endl;
}

void handle_get(http_request request)
{
   TRACE(U("\nhandle GET\n"));

   auto answer = json::value::object();

   for (auto const & p : dictionary)
   {
      answer[p.first] = json::value::string(p.second);
   }

   display_json(json::value::null(), U("R: "));
   display_json(answer, U("S: "));

   request.reply(status_codes::OK, answer);
}

int main()
{
   http_listener listener(U("http://localhost/restdemo"));

   listener.support(methods::GET, bind(&handle_get, this, std::placeholders::_1));
   listener.support(methods::POST, bind(&handle_post, this, std::placeholders::_1));
   listener.support(methods::PUT, bind(&handle_put, this, std::placeholders::_1));
   listener.support(methods::DEL, bind(&handle_del, this, std::placeholders::_1));

   try
   {
      listener
         .open()
         .then([&listener](){TRACE(L"\nstarting to listen\n");})
         .wait();

      while (true);
   }
   catch (exception const & e)
   {
      wcout << e.what() << endl;
   }

   return 0;
}
```

#### get

##### function: handle_get()

- answer에 요청에 대한 json::value 데이터 저장
- request.reply(status_codes::OK, json::value::object(answer));를 이용한 response

```cpp
void handle_request(
   http_request request,
   function<void(json::value const &, json::value &)> action)
{
   auto answer = json::value::object();

   request
      .extract_json()
      .then([&answer, &action](pplx::task<json::value> task) {
         try
         {
            auto const & jvalue = task.get();
            display_json(jvalue, U("R: "));

            if (!jvalue.is_null())
            {
               action(jvalue, answer);
            }
         }
         catch (http_exception const & e)
         {
            wcout << e.what() << endl;
         }
      })
      .wait();

   
   display_json(answer, U("S: "));

   request.reply(status_codes::OK, answer);
}
```

#### post, put, del

##### function: handle_request()

- OST, PUT, DEL methods는 GET 방식보다 복잡한 구조로 되어있음
- handle_request() 함수는 request된 json 값을 처리하고, response할 json 값을 만드는 요청을 처리하기 위한 일반적인 메소드 함수
- POST, PUT, DEL methods handler는 handle_request()를 이용하여 처리함.

```cpp
void handle_request(
   http_request request,
   function<void(json::value const &, json::value &)> action)
{
   auto answer = json::value::object();

   request
      .extract_json()
      .then([&answer, &action](pplx::task<json::value> task) {
         try
         {
            auto const & jvalue = task.get();
            display_json(jvalue, U("R: "));

            if (!jvalue.is_null())
            {
               action(jvalue, answer);
            }
         }
         catch (http_exception const & e)
         {
            wcout << e.what() << endl;
         }
      })
      .wait();

   
   display_json(answer, U("S: "));

   request.reply(status_codes::OK, answer);
}
```

##### function: handle_post()

```cpp
void handle_post(http_request request)
{
   TRACE("\nhandle POST\n");

   handle_request(
      request,
      [](json::value const & jvalue, json::value & answer)
   {
      for (auto const & e : jvalue.as_array())
      {
         if (e.is_string())
         {
            auto key = e.as_string();
            auto pos = dictionary.find(key);

            if (pos == dictionary.end())
            {
               answer[key] = json::value::string(U("<nil>"));
            }
            else
            {
               answer[pos->first] = json::value::string(pos->second);
            }
         }
      }
   });
}
```

##### function: handle_put()

```cpp
void handle_put(http_request request)
{
   TRACE("\nhandle PUT\n");

   handle_request(
      request,
      [](json::value const & jvalue, json::value & answer)
   {
      for (auto const & e : jvalue.as_object())
      {
         if (e.second.is_string())
         {
            auto key = e.first;
            auto value = e.second.as_string();

            if (dictionary.find(key) == dictionary.end())
            {
               TRACE_ACTION(U("added"), key, value);
               answer[key] = json::value::string(U("<put>"));
            }
            else
            {
               TRACE_ACTION(U("updated"), key, value);
               answer[key] = json::value::string(U("<updated>"));
            }

            dictionary[key] = value;
         }
      }
   });
}
```

##### function: handle_del()

```cpp
void handle_del(http_request request)
{
   TRACE("\nhandle DEL\n");

   handle_request(
      request,
      [](json::value const & jvalue, json::value & answer)
   {
      set<utility::string_t> keys;
      for (auto const & e : jvalue.as_array())
      {
         if (e.is_string())
         {
            auto key = e.as_string();

            auto pos = dictionary.find(key);
            if (pos == dictionary.end())
            {
               answer[key] = json::value::string(U("<failed>"));
            }
            else
            {
               TRACE_ACTION(U("deleted"), pos->first, pos->second);
               answer[key] = json::value::string(U("<deleted>"));
               keys.insert(key);
            }
         }
      }

      for (auto const & key : keys)
         dictionary.erase(key);
   });
}
```

## 2. CLIENT

- server에 HTTP request를 만드는 http_client 객체를 생성
- request method, path 및 json 값을 지정할 수 있는 오버로드 된 make_request() 함수를 가짐
- make_request()의 경우 reqeust를 디스패치하고, 받은 response를 콘솔에 표시하는 역할을 함.
- GET 매소드의 경우 json 값을 보내지 않음.

```cpp
#include <cpprest/http_client.h>
#include <cpprest/json.h>

using namespace web;
using namespace web::http;
using namespace web::http::client;

#include <iostream>
using namespace std;

void display_json(
   json::value const & jvalue, 
   utility::string_t const & prefix)
{
   cout << prefix << jvalue.serialize() << endl;
}

pplx::task<http_response> make_task_request(
   http_client & client,
   method mtd,
   json::value const & jvalue)
{
   return (mtd == methods::GET || mtd == methods::HEAD) ?
      client.request(mtd, U("/restdemo")) :
      client.request(mtd, U("/restdemo"), jvalue);
}

void make_request(
   http_client & client, 
   method mtd, 
   json::value const & jvalue)
{
   make_task_request(client, mtd, jvalue)
      .then([](http_response response)
      {
         if (response.status_code() == status_codes::OK)
         {
            return response.extract_json();
         }
         return pplx::task_from_result(json::value());
      })
      .then([](pplx::task<json::value> previousTask)
      {
         try
         {
            display_json(previousTask.get(), U("R: "));
         }
         catch (http_exception const & e)
         {
            cout << e.what() << endl;
         }
      })
      .wait();
}

int main()
{
   http_client client(U("http://localhost:9090"));

   auto putvalue = json::value::object();
   putvalue[U("one")] = json::value::string(U("100"));
   putvalue[U("two")] = json::value::string(U("200"));

   cout << U("\nPUT (add values)\n");
   display_json(putvalue, U("S: "));
   make_request(client, methods::PUT, putvalue);

   auto getvalue = json::value::array();
   getvalue[0] = json::value::string(U("one"));
   getvalue[1] = json::value::string(U("two"));
   getvalue[2] = json::value::string(U("three"));

   cout << U("\nPOST (get some values)\n");
   display_json(getvalue, U("S: "));
   make_request(client, methods::POST, getvalue);

   auto delvalue = json::value::array();
   delvalue[0] = json::value::string(U("one"));

   cout << U("\nDELETE (delete values)\n");
   display_json(delvalue, U("S: "));
   make_request(client, methods::DEL, delvalue);

   cout << U("\nPOST (get some values)\n");
   display_json(getvalue, U("S: "));
   make_request(client, methods::POST, getvalue);

   auto nullvalue = json::value::null();

   cout << U("\nGET (get all values)\n");
   display_json(nullvalue, U("S: "));
   make_request(client, methods::GET, nullvalue);

   return 0;
}
```

## 3. RESULT

<img src="/assets/img/posts/240202_cpprestsdk_result.png">

## 4. uri 분석에 따른 처리

```cpp
// GET 요청을 처리하는 handle_get 함수
void handle_get(http_request request){
  // 전체 uri는 request.request_uri().path() 메서드로 접근할 수 있음.
  auto path = request.request_uri().path();
  
  // path를 uri::split_path() 메서드를 이용하여 '/'를 구분자로 하여 parsing할 수 있음.
  // 리턴 타입 : std::vector<utility::string_t>
  auto http_get_vars = uri::split_path(path);
  
  // vector 컨테이너에 uri 순서대로 담김.
  // 예를들어 {SERVER_URL}/api/query/robot/{robot_id} 인 uri 요청이 왔을 때,
  // URL을 다음부터 0번째는 "api", 1번째는 "query", 2번째는 "robot", 3번째는 "{robot_id}" 순이 된다.
  std::string query = http_get_vars[1];
  json::value answer = json::value::object();

  if(query == "query"){
    std::string category = http_get_vars[2];
    if(category == "robot"){
      // ... robot에 대한 query 처리 ...
    }
    if(category == "task"){
      // ... task에 대한 query 처리 ...
    }
  }
  
  request.reply(status_codes::OK, answer);
}
```

* request에 query가 있는 경우

```cpp
// GET 요청을 처리하는 handle_get 함수
void handle_get(http_request request){
  // 전체 uri는 request.request_uri().path() 메서드로 접근할 수 있음.
  auto path = request.request_uri().path();
  // path를 uri::split_path() 메서드를 이용하여 '/'를 구분자로 하여 parsing할 수 있음.
  // 리턴 타입 : std::vector<utility::string_t>
  auto http_get_vars = uri::split_path(path);
  // vector 컨테이너에 uri 순서대로 담김.
  // 예를들어 {SERVER_URL}/api/query/robot?robotId={robot_id} 인 uri 요청이 왔을 때,
  // URL을 다음부터 0번째는 "api", 1번째는 "query", 2번째는 "robot" 순이 된다.
    
  // path() 메소드에는 query가 포함되지 않음
  // query를 가져오려면 query() 메소드를 사용해야 함
  auto query = request.request_uri().query();
  // query를 uri::split_query() 메서드롤 이용하여 uri에 포함된 query를 parsing할 수 있음.
  auto id = uri::split_query(query);
  // map 컨테이너처럼 query가 담김.
  // 예를들어 {SERVER_URL}/api/query/robot?robotId={robot_id} 인 uri 요청이 왔을 때,
  // query는 robotId={robot_id}이며,
  // id["robotId"]에 {robot_id}가 담긴다.
  
  std::string cmd = http_get_vars[1];
  json::value answer = json::value::object();

  if(cmd == "query"){
    std::string category = http_get_vars[2];
    if(category == "robot"){
      std::string robot_id = id["robotId"];
      // ... robot에 대한 query 처리 ...
    }
    if(category == "task"){
      std::string task_id = id["taskId"];
      // ... task에 대한 query 처리 ...
    }
  }
  
  request.reply(status_codes::OK, answer);
}
```

## 5. cpprestsdk status_codes 종류

* usr/include/cpprest/details/http_constants.dat에 정의

```
#ifdef _PHRASES
DAT(Continue,              100, _XPLATSTR("Continue"))
DAT(SwitchingProtocols,    101, _XPLATSTR("Switching Protocols"))
DAT(OK,                    200, _XPLATSTR("OK"))
DAT(Created,               201, _XPLATSTR("Created"))
DAT(Accepted,              202, _XPLATSTR("Accepted"))
DAT(NonAuthInfo,           203, _XPLATSTR("Non-Authoritative Information"))
DAT(NoContent,             204, _XPLATSTR("No Content"))
DAT(ResetContent,          205, _XPLATSTR("Reset Content"))
DAT(PartialContent,        206, _XPLATSTR("Partial Content"))
DAT(MultiStatus,           207, _XPLATSTR("Multi-Status"))
DAT(AlreadyReported,       208, _XPLATSTR("Already Reported"))
DAT(IMUsed,                226, _XPLATSTR("IM Used"))
DAT(MultipleChoices,       300, _XPLATSTR("Multiple Choices"))
DAT(MovedPermanently,      301, _XPLATSTR("Moved Permanently"))
DAT(Found,                 302, _XPLATSTR("Found"))
DAT(SeeOther,              303, _XPLATSTR("See Other"))
DAT(NotModified,           304, _XPLATSTR("Not Modified"))
DAT(UseProxy,              305, _XPLATSTR("Use Proxy"))
DAT(TemporaryRedirect,     307, _XPLATSTR("Temporary Redirect"))
DAT(PermanentRedirect,     308, _XPLATSTR("Permanent Redirect"))
DAT(BadRequest,            400, _XPLATSTR("Bad Request"))
DAT(Unauthorized,          401, _XPLATSTR("Unauthorized"))
DAT(PaymentRequired,       402, _XPLATSTR("Payment Required"))
DAT(Forbidden,             403, _XPLATSTR("Forbidden"))
DAT(NotFound,              404, _XPLATSTR("Not Found"))
DAT(MethodNotAllowed,      405, _XPLATSTR("Method Not Allowed"))
DAT(NotAcceptable,         406, _XPLATSTR("Not Acceptable"))
DAT(ProxyAuthRequired,     407, _XPLATSTR("Proxy Authentication Required"))
DAT(RequestTimeout,        408, _XPLATSTR("Request Time-out"))
DAT(Conflict,              409, _XPLATSTR("Conflict"))
DAT(Gone,                  410, _XPLATSTR("Gone"))
DAT(LengthRequired,        411, _XPLATSTR("Length Required"))
DAT(PreconditionFailed,    412, _XPLATSTR("Precondition Failed"))
DAT(RequestEntityTooLarge, 413, _XPLATSTR("Request Entity Too Large"))
DAT(RequestUriTooLarge,    414, _XPLATSTR("Request Uri Too Large"))
DAT(UnsupportedMediaType,  415, _XPLATSTR("Unsupported Media Type"))
DAT(RangeNotSatisfiable,   416, _XPLATSTR("Requested range not satisfiable"))
DAT(ExpectationFailed,     417, _XPLATSTR("Expectation Failed"))
DAT(MisdirectedRequest,    421, _XPLATSTR("Misdirected Request"))
DAT(UnprocessableEntity,   422, _XPLATSTR("Unprocessable Entity"))
DAT(Locked,                423, _XPLATSTR("Locked"))
DAT(FailedDependency,      424, _XPLATSTR("Failed Dependency"))
DAT(UpgradeRequired,       426, _XPLATSTR("Upgrade Required"))
DAT(PreconditionRequired,  428, _XPLATSTR("Precondition Required"))
DAT(TooManyRequests,       429, _XPLATSTR("Too Many Requests"))
DAT(RequestHeaderFieldsTooLarge,  431, _XPLATSTR("Request Header Fields Too Large"))
DAT(UnavailableForLegalReasons,   451, _XPLATSTR("Unavailable For Legal Reasons"))
DAT(InternalError,         500, _XPLATSTR("Internal Error"))
DAT(NotImplemented,        501, _XPLATSTR("Not Implemented"))
DAT(BadGateway,            502, _XPLATSTR("Bad Gateway"))
DAT(ServiceUnavailable,    503, _XPLATSTR("Service Unavailable"))
DAT(GatewayTimeout,        504, _XPLATSTR("Gateway Time-out"))
DAT(HttpVersionNotSupported, 505, _XPLATSTR("HTTP Version not supported"))
DAT(VariantAlsoNegotiates, 506, _XPLATSTR("Variant Also Negotiates"))
DAT(InsufficientStorage,   507, _XPLATSTR("Insufficient Storage"))
DAT(LoopDetected,          508, _XPLATSTR("Loop Detected"))
DAT(NotExtended,           510, _XPLATSTR("Not Extended"))
DAT(NetworkAuthenticationRequired, 511, _XPLATSTR("Network Authentication Required"))
#endif // _PHRASES
```
## 참고문헌

- [microsoft/cpprestsdk](https://github.com/Microsoft/cpprestsdk/wiki)
- [C++ Rest SDK](https://microsoft.github.io/cpprestsdk/namespaces.html)
- [Revisited: Full-fledged client-server example with C++ REST SDK 2.10](https://mariusbancila.ro/blog/2017/11/19/revisited-full-fledged-client-server-example-with-c-rest-sdk-2-10/)
- [Modern C++ micro-service implementation + REST API](https://medium.com/@ivan.mejia/modern-c-micro-service-implementation-rest-api-b499ffeaf898)
- [Modern C++ micro-serivce + REST API, Part II](https://medium.com/@ivan.mejia/modern-c-micro-serivce-rest-api-part-ii-7be067440ca8)
- [HTTP Status Codes - REST API Tutorial](https://restfulapi.net/http-status-codes/)