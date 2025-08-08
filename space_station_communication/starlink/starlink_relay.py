
import asyncio
import websockets

clients = set()

async def relay_handler(ws):
    clients.add(ws)
    try:
        async for msg in ws:
            # Broadcast to all other clients
            for client in clients:
                if client != ws:
                    await client.send(msg)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.remove(ws)

async def main():
    print("Starlink relay server running on ws://localhost:8080")
    async with websockets.serve(relay_handler, "0.0.0.0", 8080):
        await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
