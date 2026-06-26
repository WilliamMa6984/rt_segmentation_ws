import pstats

with open('profile.txt', 'w') as file:
    profile = pstats.Stats('profile', stream=file)
    profile.sort_stats(pstats.SortKey.CUMULATIVE)
    profile.print_stats()
    file.close()