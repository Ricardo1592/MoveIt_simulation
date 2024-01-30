import json


class JSONFileManager:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = self.load_data()

    def load_data(self):
        with open(self.file_path, 'r') as file:
            return json.load(file)

    def save_data(self):
        with open(self.file_path, 'w') as file:
            json.dump(self.data, file, indent=4)

    def update_registry(self, key, value):
        self.data[key] = value
        self.save_data()

    def delete_registry(self, key):
        del self.data[key]
        self.save_data()

    def read_registry(self, key):
        return self.data.get(key, None)

    def add_registry(self, key, value):
        self.data[key] = value
        self.save_data()

    def append_field(self, key, field, value):
        if key in self.data and isinstance(self.data[key], dict):
            self.data[key][field] = value
            self.save_data()
        else:
            raise ValueError(f"Chave '{key}' não encontrada.")

    def remove_field(self, key, field):
        if key in self.data and isinstance(self.data[key], dict) and field in self.data[key]:
            del self.data[key][field]
            self.save_data()
        else:
            raise ValueError(f"Campo '{field}' não encontrado no registro '{key}'.")
