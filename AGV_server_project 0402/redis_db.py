import redis
import os

def get_redis_connection():
    return redis.Redis(
        host=os.getenv("REDIS_HOST", "localhost"),
        port=int(os.getenv("REDIS_PORT", 6379)),
        password=os.getenv("REDIS_PASSWORD", "agvtestserverproject0402"),
        decode_responses=True
    )
