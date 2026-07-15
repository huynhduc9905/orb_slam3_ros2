export interface TrackingImageProps {
  url?: string;
}

export function TrackingImage({ url }: TrackingImageProps) {
  return (
    <section className="panel-section tracking-image" data-testid="tracking-image">
      <h2>Tracking image</h2>
      {url ? (
        <img src={url} alt="ORB-SLAM3 tracking frame" />
      ) : (
        <div className="placeholder">No tracking image</div>
      )}
    </section>
  );
}
