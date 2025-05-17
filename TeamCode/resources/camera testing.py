def getSlidePosition(pixels):
    m = (650 - 400)/(230 + 224)
    return m * (pixels + 224) + 400

print(getSlidePosition(300))
