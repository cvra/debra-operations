from homing_handler import Indexer

indexer = Indexer()
indexer.add(['left-shoulder', 'left-elbow', 'left-wrist'])
print(indexer.start())
