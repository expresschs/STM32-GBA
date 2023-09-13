#ifndef _COMMON_H_
#define _COMMON_H_

#define RET_OK          0
#define RET_FAILED      1            /* 一般错误 */
#define RET_ERROR       RET_FAILED   /* 一般错误 */
#define RET_OVERFLOW    2            /* 溢出 */
#define RET_NOT_FOUND   3            /* 请求对象未找到 */
#define RET_PARAMETER   4            /* 参数错误 */
#define RET_PARSER      5            /* 解析错误 */
#define RET_PROTOCOL    6            /* 协议错误 */
#define RET_TIMEOUT     7            /* 超时 */
#define RET_AUTH        8            /* 认证失败 */
#define RET_NETWORK     9            /* 网络未连接 */
#define RET_NO_RESPONSE 10           /* 未响应 */

#define BOOL_TRUE       1
#define BOOL_FALSE      0

#define inline __inline

#endif

