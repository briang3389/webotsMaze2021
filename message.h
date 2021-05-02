//message.h: deals with messaging the receiver for victims and stuff

char message[9] = "";

void sendMessage()
{
  cout << "Sending message" << endl;
  emitter->send(message, 9);
}

void changeMessage(int PosX, int PosZ, char type)
{
  memcpy(&message[0], &PosX, 4);
  memcpy(&message[4], &PosZ, 4);
  message[8] = type;
}
