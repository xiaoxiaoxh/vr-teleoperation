import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel
from typing import List

class Mes(BaseModel):
    q:List[float]
    isTracking:int
    pos:List[float]
    quat:List[float]

app = FastAPI()

@app.post("/get")
def get(mes:Mes):
    print(mes)
    return {'status':'ok'}
    

if __name__ == "__main__":
    uvicorn.run(app,host='172.17.41.123',port=8000)
