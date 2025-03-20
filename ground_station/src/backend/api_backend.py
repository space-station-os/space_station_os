from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pymongo import MongoClient
from dotenv import load_dotenv
import os

load_dotenv()

app = FastAPI()

# Allow CORS for React frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust for security later if needed
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Connect to MongoDB
client = MongoClient(os.getenv('MONGO_URI', 'mongodb://localhost:27017'))
db = client["ssos_demo_db"]

@app.get("/api/mongo/latest")
def get_latest_data(limit: int = 50):
    # Retrieve the latest entries sorted by timestamp
    imu_docs = list(db.imu.find().sort("timestamp", -1).limit(limit))

    # Convert ObjectId to string
    for doc in imu_docs:
        doc["_id"] = str(doc["_id"])

    return imu_docs
