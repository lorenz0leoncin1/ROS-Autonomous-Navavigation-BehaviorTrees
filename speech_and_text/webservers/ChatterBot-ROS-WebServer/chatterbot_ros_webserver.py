import websockets
from websockets.exceptions import ConnectionClosedError
from chatterbot import ChatBot
from asyncio import get_event_loop, Future
from os import environ

#ChatterBot WebServer listening for answers and sending responses back

async def chatterbot_ros(websocket, path):
    bot = ChatBot('ChatterBot for ROS', 
        logic_adapters=[
            {
                'import_path': 'chatterbot.logic.BestMatch',
                'default_response': 'I am sorry, but I didn\'t understand.',
                'maximum_similarity_threshold': 0.90
            },
            'chatterbot.logic.MathematicalEvaluation',
            'chatterbot.logic.TimeLogicAdapter',
        ],
    )
    try:
        async for message in websocket:
            try:
                answer = bot.get_response(message)  #answer is a Statement 
            except(IndexError):
                bot_message = "Your message doesn't match with any logic adapter."
                await websocket.send(bot_message)
                return      
            print(answer.text)
            await websocket.send(answer.text.replace('AM', '').replace('PM', ''))      
    #When the client left suddenly
    except (ConnectionClosedError):
        print("Connection with Client interrupted")
        return  


# Starting the WebServer Service
async def main():
    ip_addr = "0.0.0.0"
    port = environ['PORT']
    print("Serving on {}:{}".format(ip_addr, port))
    async with websockets.serve(chatterbot_ros, ip_addr, port, ping_timeout=None):
        await Future()  # run forever

loop = get_event_loop()
loop.run_until_complete(main())