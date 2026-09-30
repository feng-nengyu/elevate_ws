import os
import tos

def upload_file(key = "test.png", file_name = "test.jpg") -> str:
# 从环境变量获取 AK 和 SK 信息。
    ak = os.environ.get("TOS_AK")
    if not ak:
        raise RuntimeError("需要设置 TOS_AK 环境变量")
    sk = os.environ.get("TOS_SK")
    if not sk:
        raise RuntimeError("需要设置 TOS_SK 环境变量")
    endpoint = os.environ.get("TOS_ENDPOINT")
    region = os.environ.get("TOS_REGION")
    bucket = os.environ.get("TOS_BUCKET")
    if not all((endpoint, region, bucket)):
        raise RuntimeError(
            "需要设置 TOS_ENDPOINT、TOS_REGION 和 TOS_BUCKET 环境变量"
        )
    
    try:
        # 创建 TosClientV2 对象，对桶和对象的操作都通过 TosClientV2 实现
        client = tos.TosClientV2(ak, sk, endpoint, region)
        # 将本地文件上传到目标桶中
        # file_name为本地文件的完整路径。
        client.put_object_from_file(bucket, key, file_name)
    except tos.exceptions.TosClientError as e:
        # 操作失败，捕获客户端异常，一般情况为非法请求参数或网络异常
        print('fail with client error, message:{}, cause: {}'.format(e.message, e.cause))
    except tos.exceptions.TosServerError as e:
        # 操作失败，捕获服务端异常，可从返回信息中获取详细错误信息
        print('fail with server error, code: {}'.format(e.code))
        # request id 可定位具体问题，强烈建议日志中保存
        print('error with request id: {}'.format(e.request_id))
        print('error with message: {}'.format(e.message))
        print('error with http code: {}'.format(e.status_code))
        print('error with ec: {}'.format(e.ec))
        print('error with request url: {}'.format(e.request_url))
    except Exception as e:
        print('fail with unknown error: {}'.format(e))
        
    return client.generate_presigned_url("GET", bucket, key)

if __name__ == "__main__":
    print(upload_file())

