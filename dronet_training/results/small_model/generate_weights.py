from keras.models import Model
from keras.layers import Input, Conv2D, GlobalAveragePooling2D, Dense

# Build the minimal model
def build_minimal_model():
    input_layer = Input(shape=(200, 200, 1), name='input_1')
    x = Conv2D(16, (3, 3), strides=2, padding='same', activation='relu', name='conv1')(input_layer)
    x = Conv2D(32, (3, 3), strides=2, padding='same', activation='relu', name='conv2')(x)
    x = Conv2D(32, (3, 3), strides=2, padding='same', activation='relu', name='conv3')(x)
    x = GlobalAveragePooling2D(name='gap')(x)
    
    out_collision = Dense(1, activation='sigmoid', name='dense_collision')(x)
    out_steering = Dense(1, activation='tanh', name='dense_steering')(x)

    return Model(inputs=input_layer, outputs=[out_steering, out_collision])

# Instantiate the model
model = build_minimal_model()

# Load weights from DroNet-style h5 file, skip layers that don't match
model.load_weights('results/small_model/best_weights.h5', by_name=True, skip_mismatch=True)

# Save the adjusted weights that match this minimal model
model.save_weights('results/small_model/adjusted_minimal_model_weights.h5')
print("✅ Weights adjusted and saved.")
